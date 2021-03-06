// Copyright 2009-2015 Sandia Corporation. Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
// 
// Copyright (c) 2009-2015, Sandia Corporation
// All rights reserved.
// 
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.


#ifndef COMPONENTS_TRIG_CPU_BARRIER_RECDBL_TRIGGERED_H
#define COMPONENTS_TRIG_CPU_BARRIER_RECDBL_TRIGGERED_H

#include "sst/elements/portals4_sm/trig_cpu/application.h"
#include "sst/elements/portals4_sm/trig_cpu/trig_cpu.h"
#include "sst/elements/portals4_sm/trig_cpu/portals.h"

namespace SST {
namespace Portals4_sm {


class barrier_recdbl_triggered :  public application {
public:
    barrier_recdbl_triggered(trig_cpu *cpu) : application(cpu), init(false), algo_count(0)
    {
        int adj;

        ptl = cpu->getPortalsHandle();

        my_levels = -1;
        for (adj = 0x1; adj <= num_nodes ; adj  <<= 1) { my_levels++; } adj = adj >> 1;
        if (adj != num_nodes) {
            printf("recursive_doubling requires power of 2 nodes (%d)\n",
                   num_nodes);
            exit(1);
        }

        my_level_ct_hs.resize(my_levels);
        my_level_me_hs.resize(my_levels);
    }

    bool
    operator()(Event *ev)
    {
        ptl_md_t md;
        ptl_me_t me;

        crBegin();

        if (!init) {
            md.start = NULL;
            me.length = 0;
            md.eq_handle = PTL_EQ_NONE;
            md.ct_handle = PTL_CT_NONE;
            ptl->PtlMDBind(md, &my_md_h);

            for (i = 0 ; i < my_levels ; ++i) {
                ptl->PtlCTAlloc(PTL_CT_OPERATION, my_level_ct_hs[i]);

                me.start = NULL;
                me.length = 0;
                me.match_bits = i;
                me.ignore_bits = 0;
                me.ct_handle = my_level_ct_hs[i];
                ptl->PtlMEAppend(0, me, PTL_PRIORITY_LIST, NULL, 
                                 my_level_me_hs[i]);
            }

            init = true;
            crReturn();
	    start_noise_section();
        }

        // 200ns startup time
        start_time = cpu->getCurrentSimTimeNano();
        cpu->addBusyTime("200ns");
        crReturn();

        algo_count++;

        ptl->PtlEnableCoalesce();
        crReturn();

        // start the trip
        ptl->PtlPut(my_md_h, 0, 0, 0, my_id, 0, 0, 0, NULL, 0);
        crReturn();
        ptl->PtlPut(my_md_h, 0, 0, 0, my_id ^ 0x1, 0, 0, 0, NULL, 0);
        crReturn();

        for (i = 1  ; i < my_levels ; ++i) {
            ptl->PtlTriggeredPut(my_md_h, 0, 0, 0, my_id, 0,
                                 i, 0, NULL, 0, my_level_ct_hs[i - 1], algo_count * 2);
            crReturn();
            ptl->PtlTriggeredPut(my_md_h, 0, 0, 0, my_id ^ (0x1 << i) , 0,
                                 i, 0, NULL, 0, my_level_ct_hs[i - 1], algo_count * 2);
            crReturn();
        }

        ptl->PtlDisableCoalesce();
        crReturn();

        while (!ptl->PtlCTWait(my_level_ct_hs[my_levels - 1], algo_count * 2)) {
            crReturn(); 
        }
        crReturn(); 

        trig_cpu::addTimeToStats(cpu->getCurrentSimTimeNano()-start_time);

        crFinish();
        return true;
    }

private:
    barrier_recdbl_triggered();
    barrier_recdbl_triggered(const application& a);
    void operator=(barrier_recdbl_triggered const&);

    portals *ptl;
    SimTime_t start_time;
    int i;
    int my_levels;
    bool init;

    std::vector<ptl_handle_ct_t> my_level_ct_hs;
    std::vector<ptl_handle_me_t> my_level_me_hs;
    ptl_handle_md_t my_md_h;

    uint64_t algo_count;
};

}
}

#endif // COMPONENTS_TRIG_CPU_BARRIER_RECDBL_TRIGGERED_H

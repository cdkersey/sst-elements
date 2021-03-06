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


#ifndef COMPONENTS_TRIG_CPU_PORTALS_H
#define COMPONENTS_TRIG_CPU_PORTALS_H

#include <vector>
#include <list>
#include <map>
#include <queue>

#include "portals_types.h"
#include <sst/core/sst_types.h>

#include "sst/elements/portals4_sm/trig_nic/trig_nic_event.h"

namespace SST {
namespace Portals4_sm {

class trig_cpu;

class portals {

public:
    void PtlPTAlloc(unsigned int options, ptl_handle_eq_t eq_handle,
		   ptl_pt_index_t pt_index_req, ptl_pt_index_t *pt_index );

    void PtlEQAlloc(ptl_size_t count, ptl_handle_eq_t *eq_handle);
    void PtlEQFree(ptl_handle_eq_t eq_handle);
    void PtlEQGet(ptl_handle_eq_t eq_handle, ptl_event_t *event);
    bool PtlEQWait(ptl_handle_eq_t eq_handle, ptl_event_t *event);
    bool PtlEQPoll(int* return_value, ptl_handle_eq_t* eq_handles, unsigned int size, ptl_time_t timeout,
		   ptl_event_t* event, unsigned int* which);
    bool PtlEQPoll(int* return_value, ptl_handle_eq_t eq_handle, ptl_time_t timeout,
		   ptl_event_t* event);
    
    
    void PtlCTAlloc(ptl_ct_type_t ct_type, ptl_handle_ct_t& ct_handle); 
    void PtlCTFree(ptl_handle_ct_t ct_handle);
    void PtlCTSet(ptl_handle_ct_t ct_handle, ptl_ct_event_t new_ct);
    bool PtlCTWait(ptl_handle_ct_t ct_handle, ptl_size_t test);
    // Not in spec, need for internal
    bool PtlCTCheckThresh(ptl_handle_ct_t ct_handle, ptl_size_t test);
    // Implemented different than spec
    void PtlCTInc(ptl_handle_ct_t ct_handle, /*ptl_ct_event_t*/ ptl_size_t increment);
    void PtlCTGet(ptl_handle_ct_t ct_handle, ptl_ct_event_t* event);
  
    void PtlMDBind(ptl_md_t md, ptl_handle_md_t *md_handle);
    void PtlMDRelease(ptl_handle_md_t md_handle);
  
    void PtlMEAppend(ptl_pt_index_t pt_index, ptl_me_t me, ptl_list_t ptl_list, 
                     void *user_ptr, ptl_handle_me_t& me_handle);
    void PtlMEUnlink(ptl_handle_me_t me_handle); 

    void PtlPut(ptl_handle_md_t md_handle, ptl_size_t local_offset, 
                ptl_size_t length, ptl_ack_req_t ack_req, 
                ptl_process_t target_id, ptl_pt_index_t pt_index,
                ptl_match_bits_t match_bits, ptl_size_t remote_offset, 
                void *user_ptr, ptl_hdr_data_t hdr_data); 

    void PtlAtomic(ptl_handle_md_t md_handle, ptl_size_t local_offset,
                   ptl_size_t length, ptl_ack_req_t ack_req,
                   ptl_process_t target_id, ptl_pt_index_t pt_index,
                   ptl_match_bits_t match_bits, ptl_size_t remote_offset,
                   void *user_ptr, ptl_hdr_data_t hdr_data,
                   ptl_op_t operation, ptl_datatype_t datatype);

    void PtlTriggeredPut(ptl_handle_md_t md_handle, ptl_size_t local_offset, 
                         ptl_size_t length, ptl_ack_req_t ack_req, 
                         ptl_process_t target_id, ptl_pt_index_t pt_index,
                         ptl_match_bits_t match_bits, ptl_size_t remote_offset, 
                         void *user_ptr, ptl_hdr_data_t hdr_data,
                         ptl_handle_ct_t trig_ct_handle, ptl_size_t threshold);

    // This version not used.  Use Start and End TriggeredPutV with a
    // for loop that calls TriggeredPut's
    void PtlTriggeredPutV(ptl_handle_md_t md_handle, ptl_size_t local_offset, 
			  ptl_size_t length, ptl_ack_req_t ack_req, 
			  ptl_process_t* target_ids, ptl_size_t id_length, ptl_pt_index_t pt_index,
			  ptl_match_bits_t match_bits, ptl_size_t remote_offset, 
			  void *user_ptr, ptl_hdr_data_t hdr_data,
			  ptl_handle_ct_t trig_ct_handle, ptl_size_t threshold);
    
    void PtlStartTriggeredPutV(ptl_size_t id_length);
    void PtlEndTriggeredPutV();
    
    void PtlTriggeredAtomic(ptl_handle_md_t md_handle, ptl_size_t local_offset,
                            ptl_size_t length, ptl_ack_req_t ack_req,
                            ptl_process_t target_id, ptl_pt_index_t pt_index,
                            ptl_match_bits_t match_bits, ptl_size_t remote_offset,
                            void *user_ptr, ptl_hdr_data_t hdr_data,
                            ptl_op_t operation, ptl_datatype_t datatype,
                            ptl_handle_ct_t trig_ct_handle, ptl_size_t threshold);

    void PtlTriggeredCTInc(ptl_handle_ct_t ct_handle, ptl_size_t increment,
                           ptl_handle_ct_t trig_ct_handle, ptl_size_t threshold);
  
    void PtlGet ( ptl_handle_md_t md_handlde, ptl_size_t local_offset, 
		  ptl_size_t length, ptl_process_t target_id, 
		  ptl_pt_index_t pt_index, ptl_match_bits_t match_bits, 
		  void *user_ptr, ptl_size_t remote_offset ); 

    void PtlTriggeredGet ( ptl_handle_md_t md_handle, ptl_size_t local_offset, 
			   ptl_size_t length, ptl_process_t target_id, 
			   ptl_pt_index_t pt_index, ptl_match_bits_t match_bits, 
			   void *user_ptr, ptl_size_t remote_offset,
			   ptl_handle_ct_t ct_handle, ptl_size_t threshold);


    void PtlEnableCoalesce(void);
    void PtlDisableCoalesce(void);

    portals(trig_cpu* my_cpu);

    void scheduleUpdateHostCT(ptl_handle_ct_t ct_handle);
//     bool processMessage(int src, uint32_t* ptl_data);
    bool processMessage(SST::Portals4_sm::trig_nic_event* ev);
//     bool processNICOp(ptl_int_nic_op_t* op);

    bool progressPIO();
  
private:
#define MAX_PORTAL_TABLE_ENTRY 32
    ptl_entry_t* ptl_table[MAX_PORTAL_TABLE_ENTRY];

#define MAX_CT_EVENTS 32
//     ptl_int_ct_t ptl_ct_events[MAX_CT_EVENTS];
    ptl_int_ct_t ptl_ct_cpu_events[MAX_CT_EVENTS];

    // Portal Table Entries are held here.  For now, the only data
    // that matters is the event queue, so just point directly to that
    // for now.
    std::map<ptl_pt_index_t,ptl_handle_eq_t> pt_entries;

    // Event Queues
    typedef std::queue<ptl_event_t*> event_queue_t;
    std::vector< event_queue_t* > event_queues;
    
    trig_cpu* cpu;

    // Need a queue to store triggered operations that have already
    // triggered.  We can then process them once every 8ns
    std::queue<ptl_int_trig_op_t*> already_triggered_q;
  
    /*   bool ptlEventHandler(SST::Event *ev); */
    /*   std::vector<ptl_ct_event_t*> ct_events; */
  
    /*   std::map<ptl_handle_me_t,ptl_int_me_t*> me_map; */
    ptl_handle_me_t next_handle_me;

    // Data to support multi-packet PIOs
    void* pio_start;
    int pio_current_offset;
    int pio_length_rem;
    int pio_dest;
    ptl_handle_ct_t pio_ct_handle;

    bool currently_coalescing;

    // Pointer to a trig_nic_event used for TriggeredPutV
    bool putv_active;
    SST::Portals4_sm::trig_nic_event* putv_event;
    int putv_curr;
    
};

}
}

#endif // COMPONENTS_TRIG_CPU_PORTALS_H

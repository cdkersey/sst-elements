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

#include "sst_config.h"
#include "sst/core/serialization.h"
#include "pokesim.h"

#include <sst/core/params.h>
#include <sst/core/simulation.h>

#include <harp/mem.h>
#include <harp/core.h>

#include <cstring>

#include <iostream>
#include <iomanip>

using namespace SST;
using namespace SST::Interfaces;
using namespace SST::pokesim;
using namespace std;
using namespace Harp;

namespace SST {
namespace pokesim {
const int NO_WARP = -1, REQ_MAX = 20;

struct pokesim_core {
  pokesim_core(
    string archString, string coreFile, ostream &out,
    Interfaces::SimpleMem *link, bool basic = true
  ):
    arch(archString), dec(arch), mu(4096, arch.getWordSize()),
    core(arch, dec, mu, basic), mem(coreFile.c_str(), arch.getWordSize()),
    console(arch.getWordSize(), out, core), cyc(0),
    ldcount(0), stcount(0), fetchcount(0), memLink(link)
   {
    mu.attach(mem, 0);
    mu.attach(console, 0x80000000);

    for (unsigned w = 0; w < core.w.size(); ++w) {
      outstanding_loads.push_back(0);
    }
    
    for (unsigned i = 0; i < N_STAGES; ++i) pipeline[i] = NO_WARP;

    rwarps.insert(0);
    sched_q.push(0);
   }

  void show_pipeline() {
    cout << mem_table.size() << " outstanding mem reqs." << endl;
    for (unsigned i = 0; i < N_STAGES; ++i) {
      cout << ' ' << pipeline[i];
      if (pipeline[i] != NO_WARP)
	cout << '(' << core.w[pipeline[i]].activeThreads << ')';
    }
    cout << endl;
  }
  
  void tick() {
    // Advance the pipeline, starting from the end and working back.
    int wb_warp = pipeline[WRITEBACK];
    if (execd_q.empty()) {
      pipeline[WRITEBACK] = pipeline[EXEC];
      pipeline[EXEC] = pipeline[REG];
      pipeline[REG] = pipeline[PRED];
      if (fetched_q.empty()) {
        pipeline[PRED] = pipeline[FETCH];
        pipeline[FETCH] = pipeline[SCHED];
	if (!sched_q.empty()) {
	  pipeline[SCHED] = sched_q.front();
	  sched_q.pop();
	} else {
	  pipeline[SCHED] = NO_WARP;
	}
      } else {
	pipeline[PRED] = fetched_q.front();
	fetched_q.pop();
      }
    } else {
      pipeline[WRITEBACK] = execd_q.front();
      execd_q.pop();
    }

    // fetch
    if (pipeline[FETCH] != NO_WARP) {
      int wid = pipeline[FETCH];
      pipeline[FETCH] = NO_WARP;
      Warp &warp(core.w[wid]);
      Word pc(warp.pc);
      mem_id_t m = send_mem_req(pc, false);
      ++fetchcount;

      mem_table[m].wr = false;
      mem_table[m].fetch = true;
      mem_table[m].warp = wid;
      mem_table[m].timestamp = cyc;
    }

    // Perform execute
    if (pipeline[EXEC] != NO_WARP) {
      int wid = pipeline[EXEC];
      Warp &warp(core.w[wid]);
      int active_threads(warp.activeThreads);
      warp.step();
      if (active_threads) {
	for (unsigned i = 0; i < warp.memAccesses.size(); ++i) {
	  unsigned addr = warp.memAccesses[i].addr;
	  bool wr = warp.memAccesses[i].wr;
          mem_id_t m = send_mem_req(addr, wr);
	  if (!wr) {
	    pipeline[EXEC] = NO_WARP;
	    outstanding_loads[wid]++;
	    ++ldcount;
	  } else {
	    ++stcount;
	  }

	  mem_table[m].wr = wr;
	  mem_table[m].fetch = false;
	  mem_table[m].warp = wid;
          mem_table[m].timestamp = cyc;
	}
      }
    }

    if (wb_warp != NO_WARP) sched_q.push(wb_warp);

    drain_mem_queue();

    if (cyc % 1000000 == 0) show_pipeline();
    check_warps();
    ++cyc;
  }
  
  void test() {
    while (core.running()) tick();
    cout << "===========" << endl
         << cyc << ' ' << " cycles" << endl
         << ldcount << ' ' << " loads" << endl
         << stcount << ' ' << " stores" << endl
         << fetchcount << ' ' << " fetches" << endl;
    core.printStats();
  }

  void check_warps() {
    for (unsigned i = 0; i < core.w.size(); ++i) {
      if (core.w[i].activeThreads > 0 && !rwarps.count(i)) {
	rwarps.insert(i);
	sched_q.push(i);
      }	
    }

    if (pipeline[EXEC] != NO_WARP && core.w[pipeline[EXEC]].activeThreads == 0)
    {
      int wid = pipeline[EXEC];
      pipeline[EXEC] = NO_WARP;
      rwarps.erase(wid);
    }
  }
  
  /* memory access support */
  Interfaces::SimpleMem *memLink;
  typedef unsigned long mem_id_t;

  mem_id_t send_mem_req(unsigned long addr, bool wr);
  void drain_mem_queue();
  void rec_mem_req(mem_id_t id); // Memory callback

  /* timing simulator state */
  struct memtable_entry { int warp; bool wr, fetch; unsigned long timestamp; };
  map<mem_id_t, memtable_entry > mem_table;
  vector<int> outstanding_loads;
  set<int> rwarps;
  
  enum stage { SCHED, FETCH, PRED, REG, EXEC, WRITEBACK, N_STAGES };
  int pipeline[N_STAGES];
  queue<int> sched_q, fetched_q, execd_q;

  unsigned long cyc;
  
  /* harplib emulator state */
  ArchDef arch;
  WordDecoder dec;
  MemoryUnit mu;
  RamMemDevice mem;
  ConsoleMemDevice console;
  Harp::Core core;

  /* stats */
  unsigned long ldcount, stcount, fetchcount;


  /* Experimental: queue up outgoing memory requests */
  std::queue<SimpleMem::Request *> req_q;
};

pokesim_core::mem_id_t pokesim_core::send_mem_req(unsigned long addr, bool wr) {
  // Hash the address.
  // addr = (addr&(~0x3f) * 1103515245 + 12345)&0xffffffc0l | (addr & 0x3f);

  cout << "Mem req: " << std::dec << addr << endl;

  SimpleMem::Request *r = new SimpleMem::Request(
    wr ? SimpleMem::Request::Write : SimpleMem::Request::Read, addr, 4, 0
  );

  req_q.push(r);

  return r->id;
}

void pokesim_core::drain_mem_queue() {
  for (unsigned i = 0; i < REQ_MAX && !req_q.empty(); ++i) {
    // mem_table[req_q.front()->id].timestamp = cyc; ////
    memLink->sendRequest(req_q.front());
    req_q.pop();
  }
}

void pokesim_core::rec_mem_req(pokesim_core::mem_id_t m) {
  unsigned long latency(cyc - mem_table[m].timestamp);
  cout << "Got mem resp. Latency: " << latency << " cycles." << endl;

  if (mem_table[m].fetch) {
    fetched_q.push(mem_table[m].warp);
  } else {
    if (!mem_table[m].wr) {
      if (--outstanding_loads[mem_table[m].warp] == 0)
        execd_q.push(mem_table[m].warp);
    }
  }

  mem_table.erase(m);
}

// Debugging level macros similar to ones used in memHierarchy
#define _L0_ CALL_INFO,0,0
#define _L1_ CALL_INFO,1,0
#define _L2_ CALL_INFO,2,0
#define _L3_ CALL_INFO,3,0
#define _L4_ CALL_INFO,4,0
#define _L5_ CALL_INFO,5,0
#define _L6_ CALL_INFO,6,0
#define _L7_ CALL_INFO,7,0
#define _L8_ CALL_INFO,8,0
#define _L9_ CALL_INFO,9,0
#define _L10_ CALL_INFO,10,0

pokesim::pokesim(ComponentId_t id, Params &par):
  Component(id)
{
  bool mem_init_found;
  string arch_string(par.find_string("arch", "32w32/32/32/32")),
         mem_init_file(par.find_string("mem_init", "", mem_init_found)),
         clock_freq_string(par.find_string("clock_freq", "2GHz"));
  if (!mem_init_found) {
    Simulation::getSimulation()->
      getSimulationOutput().fatal(CALL_INFO, -1, "No memory init file.");
  }  

  typedef Clock::Handler<pokesim> ch;
  registerClock(clock_freq_string, new ch(this, &pokesim::clockTick));

  registerAsPrimaryComponent();
  primaryComponentDoNotEndSim();

  memLink = dynamic_cast<SimpleMem*>(
    loadModuleWithComponent("memHierarchy.memInterface", this, par)
  );

  typedef SimpleMem::Handler<pokesim> mh;
  if (!memLink->initialize("memLink",new mh(this, &pokesim::handleEvent)))
    Simulation::getSimulation()->
      getSimulationOutput().fatal(
        CALL_INFO, -1, "Unable to initialize Link memLink\n"
  );

  p = new pokesim_core(arch_string, mem_init_file, cout, memLink);
}

// Used by serialization.
pokesim::pokesim(): Component(-1) {}

void pokesim::setup() {}

void pokesim::init(unsigned phase) {}

void pokesim::finish() {
  cout << "========" << endl;
  cout << p->cyc << " cycles" << endl;
  p->core.printStats();
}


void pokesim::handleEvent(Interfaces::SimpleMem::Request *req) {
  p->rec_mem_req(req->id);
  delete req;
}

bool pokesim::clockTick(Cycle_t c) {
  if (p->core.running()) p->tick();
  else unregisterExit();

  return false;
}

}

}

BOOST_CLASS_EXPORT(SST::pokesim::pokesim);

static Component* create_pokesim(ComponentId_t id, Params &p) {
  return new pokesim::pokesim(id, p);
}

static const ElementInfoParam component_params[] = {
  {"arch", "Arch ID string", "32w32/32/32/32"},
  {"mem_init", "Memory initiialization file.", ""},
  {"clock_freq", "Clock frequency.", "2GHz"},
  {NULL, NULL, NULL}
};

static const ElementInfoPort component_ports[] = {
    {"memLink", "Main memory link", NULL},
    {NULL, NULL, NULL}
};

static const ElementInfoComponent components[] = {
  { "pokesim",
    "Fast modeling of Harmonica2 GPGPU core.",
    NULL,
    create_pokesim,
    component_params,
    component_ports,
    COMPONENT_CATEGORY_PROCESSOR
  },
  {NULL, NULL, NULL, NULL, NULL, NULL, 0}
};

extern "C" {
  ElementLibraryInfo pokesim_eli = {
    "pokesim",
    "Quick and dirty timing simulator using harplib for functional component.",
    components,
  };
}

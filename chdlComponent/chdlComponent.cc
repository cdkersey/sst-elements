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

// #define CHDL_FASTSIM // Support for new fastsim branch of CHDL
// #define CHDL_TRANS   // (buggy) support for translation in fastsim

#include "sst_config.h"
#include "sst/core/serialization.h"
#include "chdlComponent.h"
#include "atlsim.h"

#include <sst/core/params.h>
#include <sst/core/simulation.h>

#include <cstring>

#include <iostream>
#include <fstream>

using namespace SST;
using namespace SST::Interfaces;
using namespace SST::ChdlComponent;
using namespace chdl_sst;
using namespace std;

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

template <typename T> void convert(vector<bool> &v, T x) {
  for (unsigned i = 0; i < v.size(); ++i)
    v[i] = ((x & (1ull<<i)) ? 1 : 0);
}

template <typename T> void convert(T &x, vector<bool> &v) {
  x = 0;
  for (unsigned i = 0; i < v.size(); ++i)
    if (v[i])
      x |= (1ull<<i);
}

static void vecliteral(atlsim &sim, string name, unsigned long val) {
  vector<pair<string, unsigned> > inputs;
  sim.get_inputs(inputs);

  for (unsigned i = 0; i < inputs.size(); ++i) {
    if (inputs[i].first == name) {
      vector<bool> *v = new vector<bool>(inputs[i].second);
      convert(*v, val);
      sim.input(name, *v);
      break;
    }
  }

  return;
}

// This is embarrassing, but I'd rather do this than strtok_r.
static void tok(vector<char*> &out, char* in, const char* sep) {
  char *s(in);
  out.clear();
  for (unsigned i = 0; in[i]; ++i) {
    for (unsigned j = 0; sep[j]; ++j) {
      if (in[i] == sep[j]) {
        in[i] = 0;
        out.push_back(s);
        s = &in[i+1];
      }
    }
  }
  out.push_back(s);
}

chdlComponent::chdlComponent(ComponentId_t id, Params &p):
  Component(id), tog(0), stopSim(0), registered(false)
{
  debugLevel = p.find_integer("debugLevel", 1);
  int debug(p.find_integer("debug", 0));
  out.init("", debugLevel, 0, Output::output_location_t(debug));

  bool found;
  netlFile = p.find_string("netlist", "", found);
  if (!found) Simulation::getSimulation()->getSimulationOutput().fatal(CALL_INFO, -1, "No netlist specified.\n");

  clockFreq = p.find_string("clockFreq", "2GHz");
  memFile = p.find_string("memInit", "");
  core_id = p.find_integer("id", 0);
  core_count = p.find_integer("cores", 1);

  registerAsPrimaryComponent();
  primaryComponentDoNotEndSim();
  registered = true;

  memLink = dynamic_cast<SimpleMem*>(
    loadModuleWithComponent("memHierarchy.memInterface", this, p)
  );

  typedef SimpleMem::Handler<chdlComponent> mh;
  if (!memLink->initialize("memLink",new mh(this, &chdlComponent::handleEvent)))
    Simulation::getSimulation()->getSimulationOutput().fatal(CALL_INFO, -1, "Unable to initialize Link memLink\n");

  typedef Clock::Handler<chdlComponent> ch;
  registerClock(clockFreq, new ch(this, &chdlComponent::clockTick));

  string memPorts = p.find_string("memPorts", "mem", found);
  vector<char*> port_names;
  char s[80];
  strncpy(s, memPorts.c_str(), 80);
  tok(port_names, s, ",");
  unsigned next_id = 0;
  for (unsigned i = 0; i < port_names.size(); ++i)
    ports[port_names[i]] = next_id++;

  sim = new atlsim(netlFile);
  
  // Resize request and response vectors to accomodate all port IDs.
  resp_q.resize(next_id);
  responses_this_cycle.resize(next_id);
  byte_sz.resize(next_id);
  data_sz.resize(next_id);
}

// Used by serialization.
chdlComponent::chdlComponent(): Component(-1), sim(NULL) {}

chdlComponent::~chdlComponent() {
  delete sim;
}

void chdlComponent::setup() {}

void chdlComponent::init(unsigned phase) {
  if (phase == 0) {
    // Enumerate inputs/outpts.
    vector<pair<string, unsigned> > inputs, outputs;
    sim->get_inputs(inputs);
    sim->get_outputs(outputs);

    // Make input/output sizes easily searchable.
    map<string, unsigned> isize, osize;
    for (unsigned i = 0; i < inputs.size(); ++i)
      isize[inputs[i].first] = inputs[i].second;
    for (unsigned i = 0; i < outputs.size(); ++i)
      osize[outputs[i].first] = outputs[i].second;
    
    // Connect the memory ports.
    map<string, unsigned>::iterator p_it;
    for(p_it = ports.begin(); p_it != ports.end(); ++p_it) {
      const pair<string, unsigned> &p(*p_it);
      unsigned idx(p.second);
      const string &s(p.first);
      unsigned b(osize[s + "_req_contents_data_0"]),
               n(osize[s + "_req_contents_mask"]),
               a(osize[s + "_req_contents_addr"]),
               i(osize[s + "_req_contents_id"]);

      reqb.push_back(reqbin(b,n,a,i));
      reqb_copy.push_back(reqbin(b, n, a, i));
      respb.push_back(respbin(b,n,i));
      
      byte_sz[idx] = b;
      data_sz[idx] = n;
      if (n > 64) cout << "ERROR: max bytes per word (64) exceded.\n";
      if (b % 8) cout << "ERROR: byte size must be multiple of 8.\n";
      
      // The request side
      reqb[idx].ready[0] = true;
      sim->input(s + "_req_ready", reqb[idx].ready);
      sim->output(s + "_req_valid", reqb[idx].valid);
      sim->output(s + "_req_contents_wr", reqb[idx].wr);
      sim->output(s + "_req_contents_llsc", reqb[idx].llsc);
      sim->output(s + "_req_contents_addr", reqb[idx].addr);
      sim->output(s + "_req_contents_mask", reqb[idx].mask);
      sim->output(s + "_req_contents_id", reqb[idx].id);
      for (unsigned i = 0; i < n; ++i) {
        ostringstream oss; oss << s << "_req_contents_data_" << i;
	sim->output(oss.str(), reqb[idx].data[i]);
      }

      // The response side
      sim->output(s + "_resp_ready", respb[idx].ready);
      sim->input(s + "resp_valid", respb[idx].valid);
      sim->input(s + "resp_contents_wr", respb[idx].wr);
      sim->input(s + "resp_contents_llsc", respb[idx].llsc);
      sim->input(s + "resp_contents_llsc_suc", respb[idx].llsc_suc);
      sim->input(s + "resp_contents_id", respb[idx].id);
      for (unsigned i = 0; i < n; ++i) {
	ostringstream oss; oss << s << "_resp_contents_data_" << i;
	sim->input(oss.str(), respb[idx].data[i]);
      }
    }

    // Find auxiliary I/O
    for (unsigned i = 0; i < outputs.size(); ++i) {
      char s[80];
      vector<char*> t;
      strncpy(s, outputs[i].first.c_str(), 80);
      tok(t, s, "_");

      // Counters
      if (!strncmp(t[0], "counter", 80)) {
        counters[outputs[i].first] = new vector<bool>(outputs[i].second);
	sim->output(outputs[i].first, *counters[outputs[i].first]);
      } else if (
        t.size() == 2 &&
        !strncmp(t[0], "stop", 80) &&
	!strncmp(t[1], "sim", 80)
      ) {
	// TODO: Implement stop_sim
      }
    }

    for (unsigned i = 0; i < inputs.size(); ++i) {
      if (inputs[i].first == "id") {
	vecliteral(*sim, inputs[i].first, core_count);
      } else if (inputs[i].first == "cores") {
	vecliteral(*sim, inputs[i].first, core_id);
      }
    }
  } else if (phase == 2 && memFile != "") {
    ifstream m(memFile);

    unsigned limit(16*1024), addr(0);
    while (!!m && --limit) {
      char buf[1024];
      m.read(buf, 1024);
      SimpleMem::Request *req =
        new SimpleMem::Request(SimpleMem::Request::Write, addr, 1024);
      req->setPayload((unsigned char *)buf, 1024);
      memLink->sendInitData(req);
      addr += 1024;
    }
  }
}

void chdlComponent::finish() {
  unsigned long simCycle(Simulation::getSimulation()->getCurrentSimCycle());  

  for (auto &x : counters)
    out.output("CHDL %u counter \"%s\": %lu\n",
               core_id, x.first.c_str(), *x.second);

  // out.debug(_L2_, "%lu sim cycles\n", simCycle); TODO
}


void chdlComponent::handleEvent(Interfaces::SimpleMem::Request *req) {
  unsigned port = portMap[req->id];

  // TODO: valid is always cleared; we should check that the requester was
  // actually ready.
  if (responses_this_cycle[port] || respb[port].valid[0]) {
    out.debug(_L2_, "Adding an entry to resp_q[%u]. ", port);
    resp_q[port].push(req);
  } else {
    respb[port].wr[0] = req->cmd == SimpleMem::Request::WriteResp;

    respb[port].valid[0] = true;

    if (!respb[port].wr[0]) {
      if (req->addr & 0x80000000) {
	unsigned long d = mmio_rd(req->addr);
	/* TODO stuff d into respb */
      }

      unsigned idx = 0;
      for (unsigned i = 0; i < data_sz[port]; ++i) {
        unsigned long b = 0;
        for (unsigned j = 0; j < byte_sz[port]; j += 8) {
	  b <<= 8;
	  b |= req->data[idx];
	  idx++;
	}
	convert(respb[port].data[i], b);
      }
    }
    convert(respb[port].id, idMap[req->id]);

    respb[port].llsc[0] = ((req->flags & SimpleMem::Request::F_LLSC) != 0);
    respb[port].llsc_suc[0] =
      ((req->flags & SimpleMem::Request::F_LLSC_RESP) != 0);

    out.debug(_L1_, "Response arrived on port %d for req %d, wr = %d, "
                    "size = %lu, datasize = %lu, flags = %x\n",
                 int(port), int(req->id), respb[port].wr[0],
                 req->size, req->data.size(), req->flags);

    delete req;

    ++responses_this_cycle[port];
  }
}

void chdlComponent::consoleOutput(char c) {
 if (c == '\n') {
   // TODO: cycle number using SST get cycle function?
   out.output("%u OUTPUT> %s\n", core_id, outputBuffer.c_str());
   outputBuffer.clear();
 } else {
   outputBuffer = outputBuffer + c;
 }
}

bool chdlComponent::clockTick(Cycle_t c) {
  if (tog) {
    sim->tick();

    /*TODO for (unsigned i = 0; i < respb.size(); ++i) respb[i].valid[0] = 0; */
  } else {
    #if 0
    for (unsigned i = 0; i < reqb.size(); ++i) {
      if (responses_this_cycle[i] == 0 && !resp_q[i].empty()) {
        // Handle enqueued event.
        handleEvent(resp_q[i].front());
        resp_q[i].pop();
      } else if (responses_this_cycle[i] > 1) {
        out.output("ERROR: %u responses on port %u this cycle.\n",
                   responses_this_cycle[i], i);
      }
      responses_this_cycle[i] = 0;
    }
    #endif

    /* for (auto &t : tickables()[cd]) t->pre_tick(tick_arg); TODO */

    // Copy new requests
    for (unsigned i = 0; i < reqb.size(); ++i) {
      if (reqb[i].ready[0] && reqb[i].valid[0]) {
	reqb_copy[i] = reqb[i];
      }
    }
    
    // Handle requests
    for (unsigned i = 0; i < reqb.size(); ++i) {
      bool rd(reqb_copy[i].valid[0] && !reqb_copy[i].wr[0]),
           wr = 0;
      for (unsigned j = 0; j < reqb_copy[i].mask.size(); ++j)
	if (reqb_copy[i].wr[0] && reqb_copy[i].mask[j])
	  wr = 1;
      if (rd || wr) {
        int flags = (reqb_copy[i].llsc[0] ? SimpleMem::Request::F_LLSC : 0);
	
        uint64_t mask;
	convert(mask, reqb_copy[i].mask);
	uint64_t mask_lsb_only(mask&-mask), mask_lsb_pos(log2(mask_lsb_only)),
                 mask_msb_only((mask+mask_lsb_only) & -(mask+mask_lsb_only)),
                 mask_msb_pos(log2(mask_msb_only)),
                 mask_cluster((mask_lsb_only-1) ^ (mask_msb_only-1)),
                 mask_cluster_len(mask_msb_pos - mask_lsb_pos);

	mask ^= mask_cluster;
        convert(reqb_copy[i].mask, mask);
	
        unsigned b(byte_sz[i]), n(data_sz[i]),
	         req_size(wr ? b * mask_cluster_len / 8 : n * b / 8);
	unsigned long addr;
	convert(addr, reqb_copy[i].addr);
	addr = addr * n * (b / 8);

        // Generate SimpleMem Request
	SimpleMem::Request *r;
	if (wr) {
	  // Write addresses are offset by the mask LSB.
	  addr += mask_lsb_pos * b / 8;
          vector<uint8_t> dVec(n * b / 8);

	  unsigned idx = 0;
	  for (unsigned j = 0; j < mask_cluster_len; ++j) {
	    uint64_t word;
	    convert(word, reqb_copy[i].data[j]);
	    for (unsigned k = 0; k < b; k += 8) {
	      dVec[idx] = (word >> k)&0xff;
	      ++idx;
	    }
	  }

          if (addr & 0x80000000) mmio_wr(dVec, addr);
	  
	  r = new SimpleMem::Request(
	    SimpleMem::Request::Write, addr, req_size, dVec, flags
	  );
	} else {
	  r = new SimpleMem::Request(
            SimpleMem::Request::Read, addr, req_size, flags
	  );
	}

	convert(idMap[r->id], reqb_copy[i].id);
	portMap[r->id] = i;

	memLink->sendRequest(r);

	// Set valid bit based on whether req has been fully handled.
	reqb_copy[i].valid[0] = wr && (mask != 0);
	reqb[i].ready[0] = !reqb_copy[i].valid[0];
      }
    }

    // If the CPU is exiting the simulation, unregister.
    if (stopSim && registered) {
      out.output("Core %u UNREGISTERING EXIT\n", core_id);
      registered = false;
      unregisterExit();
    }
  }

  tog = !tog;
  
  return false;
}

uint32_t chdlComponent::mmio_rd(uint64_t addr) {
  uint32_t data(0);

  switch (addr) {
    case 0x88000004: data = core_id; break;
    case 0x88000008: data = core_count; break;
  }
  
  out.debug(_L2_, "MMIO RD addr = 0x%08lx, data = 0x%08x", addr, data);
  
  return data;
}

void chdlComponent::mmio_wr(uint32_t data, uint64_t addr) {
  out.debug(_L2_, "MMIO WR addr = 0x%08lx, data = 0x%08x", addr, data);

  switch (addr) {
    case 0x80000004: stopSim = true; break;
    case 0x80000008: consoleOutput(data); break;
  }
}

void chdlComponent::mmio_rd(vector<uint8_t> &d, uint64_t addr) {
  // Assume addr aligned on 4-byte boundary
  for (unsigned i = 0; i < d.size(); i += 4) {
    uint32_t x(mmio_rd(addr + i));

    for (unsigned j = 0; j < 4; ++j)
      d[i + j] = (x >> (8 * j)) & 0xff;
  }
}

void chdlComponent::mmio_wr(const vector<uint8_t> &d, uint64_t addr) {
  for (unsigned i = 0; i < d.size(); i += 4) {
    uint32_t x(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16) | d[i + 3] << 24);
    mmio_wr(x, addr + i);
  }
}

BOOST_CLASS_EXPORT(SST::ChdlComponent::chdlComponent);

static Component* create_chdlComponent(ComponentId_t id, Params &p) {
  return new chdlComponent(id, p);
}

static const ElementInfoParam component_params[] = {
  {"clockFreq", "Clock rate", "2GHz"},
  {"netlist", "Filename of CHDL .nand netlist", ""},
  {"debug", "Destination for debugging output", "0"},
  {"debugLevel", "Level of verbosity of output", "1"},
  {"memPorts", "Memory port names, comma-separated.", "mem"},
  {"memInit", "File containing initial memory contents", ""},
  {"id", "Device ID passed to \"id\" input, if present", "0"},
  {"cores", "Max device ID plus 1.", "0"},
  {NULL, NULL, NULL}
};

static const ElementInfoPort component_ports[] = {
    {"memLink", "Main memory link", NULL},
    {NULL, NULL, NULL}
};

static const ElementInfoComponent components[] = {
  { "chdlComponent",
    "Register transfer level modeling of devices.",
    NULL,
    create_chdlComponent,
    component_params,
    component_ports,
    COMPONENT_CATEGORY_PROCESSOR
  },
  {NULL, NULL, NULL, NULL, NULL, NULL, 0}
};

extern "C" {
  ElementLibraryInfo chdlComponent_eli = {
    "chdlComponent",
    "CHDL netlist execution component.",
    components,
  };
}

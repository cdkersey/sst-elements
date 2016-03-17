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

#ifndef CHDL_COMPONENT_H
#define CHDL_COMPONENT_H

#include <string>
#include <map>
#include <queue>
#include <vector>

#include <sst/core/sst_types.h>
//#include <sst/core/serialization/element.h>
#include <assert.h>

#include <sst/core/component.h>
#include <sst/core/element.h>
#include <sst/core/event.h>
#include <sst/core/link.h>
#include <sst/core/output.h>
#include <sst/core/timeConverter.h>

#include <sst/core/interfaces/simpleMem.h>

#include "atlsim.h"

namespace SST {
class Params;

namespace ChdlComponent {
  // The simulator requires vector<bool> for all inputs and outputs, therefore
  // the memory requets and responses are handled using these data structures:
  struct reqbin {
    reqbin(unsigned b, unsigned n, unsigned a, unsigned i):
      ready(1), valid(1), wr(1), llsc(1), mask(n), id(i), addr(a),
      data(n, std::vector<bool>(b))
    {}

    std::vector<bool> ready, valid, wr, llsc, mask, id, addr;
    std::vector<std::vector<bool> > data;
  };

  struct respbin {
    respbin(unsigned b, unsigned n, unsigned i):
      ready(1), valid(1), wr(1), llsc(1), llsc_suc(1), id(i),
      data(n, std::vector<bool>(b))
    {}

    std::vector<bool> ready, valid, wr, id, llsc, llsc_suc;
    std::vector<std::vector<bool> > data;
  };

  class chdlComponent : public Component {
  public:
    chdlComponent(ComponentId_t id, Params &p);
    chdlComponent();
    ~chdlComponent();

    void setup();
    void init(unsigned phase);
    void finish();

    friend class boost::serialization::access;
    template <class Archive>
      void save(Archive &ar, const unsigned version) const
    {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Component);
      ar & BOOST_SERIALIZATION_NVP(memLink);
      ar & BOOST_SERIALIZATION_NVP(clockFreq);
      ar & BOOST_SERIALIZATION_NVP(netlFile);
      ar & BOOST_SERIALIZATION_NVP(memFile);
      ar & BOOST_SERIALIZATION_NVP(outputBuffer);
      // ar & BOOST_SERIALIZATION_NVP(req);
      // ar & BOOST_SERIALIZATION_NVP(resp); 
      ar & BOOST_SERIALIZATION_NVP(idMap);
      ar & BOOST_SERIALIZATION_NVP(portMap);
      ar & BOOST_SERIALIZATION_NVP(ports);
      ar & BOOST_SERIALIZATION_NVP(debugLevel);
      ar & BOOST_SERIALIZATION_NVP(tog);
      ar & BOOST_SERIALIZATION_NVP(core_id);
      ar & BOOST_SERIALIZATION_NVP(core_count);
      ar & BOOST_SERIALIZATION_NVP(out);
      ar & BOOST_SERIALIZATION_NVP(stopSim);
      ar & BOOST_SERIALIZATION_NVP(registered);
    }

    template <class Archive>
      void load(Archive &ar, const unsigned version)
    {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Component);
      ar & BOOST_SERIALIZATION_NVP(memLink);
      ar & BOOST_SERIALIZATION_NVP(clockFreq);
      ar & BOOST_SERIALIZATION_NVP(netlFile);
      ar & BOOST_SERIALIZATION_NVP(memFile);
      ar & BOOST_SERIALIZATION_NVP(outputBuffer);
      // ar & BOOST_SERIALIZATION_NVP(req);
      // ar & BOOST_SERIALIZATION_NVP(resp); 
      ar & BOOST_SERIALIZATION_NVP(idMap);
      ar & BOOST_SERIALIZATION_NVP(portMap);
      ar & BOOST_SERIALIZATION_NVP(ports);
      ar & BOOST_SERIALIZATION_NVP(debugLevel);
      ar & BOOST_SERIALIZATION_NVP(tog);
      ar & BOOST_SERIALIZATION_NVP(core_id);
      ar & BOOST_SERIALIZATION_NVP(core_count);
      ar & BOOST_SERIALIZATION_NVP(out);
      ar & BOOST_SERIALIZATION_NVP(stopSim);
      ar & BOOST_SERIALIZATION_NVP(registered);
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

  private:
    void consoleOutput(char c);

    void handleEvent(Interfaces::SimpleMem::Request *);
    bool clockTick(Cycle_t);

    uint32_t mmio_rd(uint64_t addr);
    void mmio_wr(uint32_t data, uint64_t addr);
    void mmio_rd(std::vector<uint8_t> &d, uint64_t addr);
    void mmio_wr(const std::vector<uint8_t> &d, uint64_t addr);

    Interfaces::SimpleMem *memLink;

    std::string clockFreq, netlFile, memFile, outputBuffer;

    std::vector<reqbin> reqb, reqb_copy;
    std::vector<respbin> respb;
    
    std::map<std::string, unsigned> ports;
    std::vector<std::queue<Interfaces::SimpleMem::Request *> > resp_q;
    std::vector<unsigned> responses_this_cycle, byte_sz, data_sz;
    std::map<unsigned long, unsigned long> idMap, portMap;
    std::map<std::string, std::vector<bool>* > counters;

    chdl_sst::atlsim *sim;
    
    int debugLevel, tog, core_id, core_count;

    Output out;
    bool stopSim, registered;
};

}}

#endif

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

#ifndef POKESIM_H
#define POKESIM_H

#include <string>
#include <map>
#include <queue>
#include <vector>

#include <harp/core.h>
#include <harp/instruction.h>

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

namespace SST {
class Params;

namespace pokesim {
  struct pokesim_core;

  class pokesim : public Component {
  public:
    pokesim(ComponentId_t id, Params &p);
    pokesim();

    void setup();
    void init(unsigned phase);
    void finish();

    friend class boost::serialization::access;
    template <class Archive>
      void save(Archive &ar, const unsigned version) const
    {}

    template <class Archive>
      void load(Archive &ar, const unsigned version)
    {}

    BOOST_SERIALIZATION_SPLIT_MEMBER()

  private:
    void handleEvent(Interfaces::SimpleMem::Request *);
    bool clockTick(Cycle_t);

    Interfaces::SimpleMem *memLink;

    Output out;

    pokesim_core *p;
};

}}

#endif

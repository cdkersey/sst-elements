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
#include "sst/elements/SysC/common/MEgenerator.h"
#include "sst/elements/SysC/common/MEanouncer.h"
using namespace SST;
using namespace SST::SysC;

/*
extern Component* create_MEGenerator(ComponentId_t _id,
                                     Params& _params);
extern ElementInfoParam MEgenerator_params[];
extern ElementInfoPort MEgenerator_ports[];
*/
extern ElementInfoComponent MEgenerator_info;
/*
extern Component* create_MEanouncer(ComponentId_t _id,
                                     Params& _params);
extern ElementInfoParam MEanouncer_params[];
extern ElementInfoPort MEanouncer_ports[];
*/
extern ElementInfoComponent MEanouncer_info;
/*
static const ElementInfoComponent components[] = {
  { "MEGenerator",
    "memEvent Generator Component",
    NULL,
    create_MEGenerator,
    MEgenerator_params,
    MEgenerator_ports,
    0
  },
  { "MEAnouncer",
    "memEvent Anouncer(reciever) Component",
    NULL,
    create_MEAnouncer,
    MEanouncer_params,
    MEanouncer_ports,
    0
  },
  {NULL,NULL,NULL,NULL,NULL,NULL,0}
};
*/
static const ElementInfoComponent components[]={
  MEgenerator_info,
  MEAnouncer_info,
  {NULL,NULL,NULL,NULL,NULL,NULL,0}
};

extern "C" {
  ElementLibraryInfo SysCTest_eli = {
    "SysCTest",
    "Components for testing SystemC",
    components,
  };
}

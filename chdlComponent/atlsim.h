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

/* atlsim -- allow threaded logic simulation */
#ifndef ATLSIM_H
#define ATLSIM_H

#include <string>
#include <vector>
#include <utility>
#include <iostream>

namespace chdl_sst {
  typedef unsigned long cycle_t;
  typedef unsigned nodeid_t;

  // Gate-level, thread-safe C++03 compatible simulator for CHDL netlists.
  class atlsim {
   public:
    // Instantiate simulator running attached netlist file.
    atlsim(std::string netlist);

    // Use the provided bit vector as an input to the simulator. Changes to
    // input values must be made prior to calling the tick() function for the
    // corresponding cycle.
    void input(std::string name, std::vector<bool> &x);
    
    // Use the provided bit vector as an output to the simulator. Tick() must be
    // called once before the values for cycle 0 are valid.
    void output(std::string name, std::vector<bool> &x);

    // Provide a vector of input names and widths.
    void get_outputs(std::vector<std::pair<std::string, unsigned> > &);
    
    // Provide a vector of output names and widths.
    void get_inputs(std::vector<std::pair<std::string, unsigned> > &);
    
    // Advance by 1 cycle.
    void tick();
  
   private:
    cycle_t now; // Current cycle

    // The instruction types as seen in the code array. The comments following
    // them describe their behavior in 
    enum op_t {
      OP_NOP,  // Do nothing- used by regs and syncmems
      OP_INV,  // inverter(input nodeid)
      OP_NAND, // nand gate(input 0 nodeid, input 1 nodeid)
      OP_TRI,  // tri-state node(# inputs, input 0 enable, input 0, ...)
      OP_INPUT // input node(index in inputs, bit position)
    };

    // Simulated SRAM write port
    struct wrport_t {
      std::vector<nodeid_t> a, d;
      nodeid_t w;
      unsigned base;
    };

    // Simulated SRAM read port
    struct rdport_t {
      std::vector<nodeid_t> a, q;
      unsigned base;
    };

    // An output from the simulator. Can be attached to external bit vector
    // using output() public member function.
    struct output_t {
      std::vector<bool> *v;
      std::vector<nodeid_t> nodes;
    };

    // An input to the simulator, of width w. Can be attached to external bit
    // vector using input() public member function.
    struct input_t {
      std::vector<bool> *v;
      unsigned w;
    };
    
    std::vector<cycle_t> evaltime;  // Timestamp of previous node evaluation

    std::vector<unsigned> code_idx, // Offset in code array for each node
                          code;     // Node implementation code

    std::vector<wrport_t> wrports; // Memory write ports
    std::vector<rdport_t> rdports; // Memory read ports
    std::vector<bool> memstate;    // State of SRAM

    // Register node ids, as pairs (d, q)
    std::vector<std::pair<nodeid_t, nodeid_t> > regs;

    // Inputs and outputs of the design. Pairs name of input/output with data
    // structure representing it.
    std::vector<std::pair<std::string, input_t> > inputs;
    std::vector<std::pair<std::string, output_t> > outputs;

    // Logic states of nodes in current and next cycle.
    std::vector<bool> state0, state1;
    std::vector<bool> *to, *from;
    
    // Read netlist from CHDL .nand file, returning number of nodes.
    unsigned read_netlist(std::istream &netl);

    // Read various sub-parts of the .nand file.
    void read_inputs(std::istream &in);
    void read_outputs(std::istream &in);
    void read_design(std::istream &in);
    void read_design_line(std::istream &in);

    // Used by the netlist reader to populate the design arrays.
    void new_node(nodeid_t id, bool default_value = false);
    void gen(std::string type,std::vector<nodeid_t> p,std::vector<nodeid_t> n); 
    void gen_syncram(unsigned abits, unsigned dbits, std::vector<nodeid_t> &);

    // Make #nodes-length vectors large enough to hold the given node ID
    void grow_vecs(unsigned idx);
    
    // Evaluate logic node
    bool eval(nodeid_t n);

    // Evaluate SRAM ports.
    void do_write(wrport_t &wp);
    void do_read(rdport_t &rp);

    // Used by do_read and do_write to compute addresses from gate values.
    unsigned to_uint(const std::vector<nodeid_t> &);
  };
}
#endif

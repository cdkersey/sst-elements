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

#include "atlsim.h"

#include <fstream>

using namespace std;
using namespace chdl_sst;

atlsim::atlsim(string filename): to(&state0), from(&state1), now(~0ul) {
  // Open netlist file and read it.
  ifstream netl(filename.c_str());
  if (!netl) throw("Invalid netlist file path.");

  unsigned n = read_netlist(netl);
    
  if (code_idx.size() != n)
    throw("Internal error: node array size mismatch.");

  // Initialize evaltime to -2
  for (unsigned i = 0; i < evaltime.size(); ++i)
    evaltime[i] = ~1ul;
}

unsigned atlsim::to_uint(const vector<nodeid_t> &v) {
  unsigned r = 0;
  for (unsigned i = 0; i < v.size(); ++i)
    if (eval(v[i]))
      r |= (1u<<i);

  return r;
}

void atlsim::do_write(wrport_t &p) {
  if (eval(p.w)) {
    unsigned a(to_uint(p.a)),    // Address
             wsize(p.d.size());  // Word size
    for (unsigned i = 0; i < wsize; ++i)
      memstate[p.base + wsize * a + i] = eval(p.d[i]);
  }
}

void atlsim::do_read(rdport_t &p) {
  unsigned a(to_uint(p.a)),   // Address value
           wsize(p.q.size()); // Word size
  for (unsigned i = 0; i < wsize; ++i)
    (*to)[p.q[i]] = memstate[p.base + wsize * a + i];
}

void atlsim::tick() {
  // Find next values for registers.
  typedef vector<pair<nodeid_t, nodeid_t> >::iterator reg_it_t;
  for (reg_it_t it = regs.begin(); it != regs.end(); ++it)
    (*from)[it->second] = eval(it->first);

  // Do memory writes
  for (unsigned i = 0; i < wrports.size(); ++i)
    do_write(wrports[i]);
  
  // Find next value for memory q nodes.
  for (unsigned i = 0; i < rdports.size(); ++i)
    do_read(rdports[i]);

  // Update outputs.
  typedef vector<pair<string, output_t> >::iterator out_it_t;
  for (out_it_t it = outputs.begin(); it != outputs.end(); ++it) {
    vector<bool> *v = (it->second.v);
    if (v)
      for (unsigned i = 0; i < v->size(); ++i)
        (*v)[i] = eval(it->second.nodes[i]);
  }

  // Swap "to" and "from" pointers.
  vector<bool> *tmp = to;
  to = from;
  from = tmp;

  // Increment current cycle.
  now++;
}

bool atlsim::eval(nodeid_t n) {
  // Only perform evaluation if the node has not been evaluated this cycle.
  if (evaltime[n] != now) {
    evaltime[n] = now;
    bool result;
    
    unsigned idx = code_idx[n];
    switch(code[idx]) {
    case OP_NOP:
      result = (*to)[n];
      break;
    case OP_INV:
      result = !eval(code[idx + 1]);
      break;
    case OP_NAND:
      result = !(eval(code[idx+1]) && eval(code[idx+2]));
      break;
    case OP_TRI: {
      unsigned n = code[idx + 1];
      for (unsigned i = 0; i < n*2; i += 2) {
        if (eval(code[idx + i + 2])) {
	  result = eval(code[idx + i + 3]);
	  break;
	}
      }
      break;
    }
      
    case OP_INPUT:
      result = (*(inputs[code[idx + 1]].second.v))[code[idx + 2]];
      break;
    };

    (*to)[n] = result;
    
    return result;
  } else {
    return (*to)[n];
  }
  
}

void atlsim::grow_vecs(unsigned idx) {
  if (code_idx.size() <= idx) {
    code_idx.resize(idx + 1);
    state0.resize(idx + 1);
    state1.resize(idx + 1);
    evaltime.resize(idx + 1);
  }
}

void atlsim::new_node(nodeid_t id, bool default_value) {
  grow_vecs(id);
  state0[id] = state1[id] = default_value;
  code_idx[id] = code.size();
}

void atlsim::read_inputs(istream &in) {
  while (in.peek() == ' ') {
    input_t input;
    input.w = 0;
    input.v = NULL;

    string name;
    in >> name;
    vector<unsigned> nodes;
    while (in.peek() != '\n') {
      nodeid_t n;
      in >> n;
      nodes.push_back(n);
    }
    in.get();

    for (unsigned i = 0; i < nodes.size(); ++i) {
      new_node(nodes[i]);

      // Format: opcode - input index - bit index
      code.push_back(OP_INPUT);
      code.push_back(inputs.size());
      code.push_back(i);

      // 1 more bit of width.
      ++(input.w);
    }

    inputs.push_back(make_pair(name, input));
  }
}

void atlsim::read_outputs(istream &in) {
  while (in.peek() == ' ') {
    string name;
    in >> name;
    vector<unsigned> nodes;
    while (in.peek() != '\n') {
      nodeid_t n;
      in >> n;
      nodes.push_back(n);
    }
    in.get();

    outputs.push_back(make_pair(name, output_t()));
    outputs.rbegin()->second.v = NULL;      
    for (unsigned i = 0; i < nodes.size(); ++i)
      outputs.rbegin()->second.nodes.push_back(nodes[i]);
  }
}

void atlsim::read_design_line(istream &in) {
  string type;
  in >> type;

  while (in.peek() == ' ') in.get();

  if (type == "") return;

  vector<unsigned> params;
  if (in.peek() == '<') {
    in.get();
    while (in.peek() != '>') {
      unsigned n;
      in >> n;
      params.push_back(n);
    }
    in.get();
  }

  vector<nodeid_t> nv;
  while (in.peek() != '\n') {
    nodeid_t n;
    in >> n;
    nv.push_back(n);
  }
  in.get();

  gen(type, params, nv);
}

void atlsim::gen
  (std::string type, std::vector<nodeid_t> params, std::vector<nodeid_t> nodes)
{
  if (type == "lit0") {
    new_node(nodes[0], false);
    code.push_back(OP_NOP);
  } else if (type == "lit1") {
    new_node(nodes[0], true);
    code.push_back(OP_NOP);
  } else if (type == "litX") {
    new_node(nodes[0], false);
    code.push_back(OP_NOP);
  } else if (type == "reg") {
    new_node(nodes[1]);
    code.push_back(OP_NOP);
    regs.push_back(make_pair(nodeid_t(nodes[0]), nodeid_t(nodes[1])));
  } else if (type == "inv") {
    new_node(nodes[1]);
    code.push_back(OP_INV);
    code.push_back(nodes[0]);
  } else if (type == "nand") {
    new_node(nodes[2]);
    code.push_back(OP_NAND);
    code.push_back(nodes[0]);
    code.push_back(nodes[1]);
  } else if (type == "tri") {
    new_node(*(nodes.rbegin()));
    code.push_back(OP_TRI);
    unsigned n = (nodes.size() - 1)/2;
    code.push_back(n);
    for (unsigned i = 0; i < n; ++i) {
      code.push_back(nodes[i*2 + 2]);
      code.push_back(nodes[i*2 + 1]);
    }
  } else if (type == "syncram") {
    gen_syncram(params[0], params[1], nodes);
  } else {
    throw("Unrecognized node type in input netlist.");
  }
}

void atlsim::gen_syncram(unsigned abits, unsigned dbits, vector<nodeid_t> &v) {
  unsigned ports = (v.size() - 1) / (abits + dbits),
           size = (1<<abits), bits = size * dbits, base = memstate.size();

  // Expand the memstate vector.
  memstate.resize(base + bits);
  
  // Create the write port.
  wrport_t wp;
  unsigned idx = 0;
  for (unsigned i = 0; i < abits; ++i) wp.a.push_back(v[idx++]);
  for (unsigned i = 0; i < dbits; ++i) wp.d.push_back(v[idx++]);
  wp.w = v[idx++];

  wrports.push_back(wp);

  // Create the read ports.
  for (unsigned i = 0; i < ports-1; ++i) {
    rdport_t rp;
    for (unsigned j = 0; j < abits; ++j) rp.a.push_back(v[idx++]);
    for (unsigned j = 0; j < dbits; ++j) {
      // The codes for memory reads are nops.
      new_node(v[idx]);
      code.push_back(OP_NOP);
      rp.q.push_back(v[idx++]);
    }
    rdports.push_back(rp);
  }
}

void atlsim::read_design(istream &in) {
  while (!!in) read_design_line(in);
}

unsigned atlsim::read_netlist(istream &in) {
  string eat;
  in >> eat; in.get();
  if (eat != "inputs") throw("'inputs' expected in netlist.\n");

  read_inputs(in);

  in >> eat; in.get();
  if (eat != "outputs") throw("'outputs' expected in netlist.\n");

  read_outputs(in);

  in >> eat; in.get();
  if (eat != "inout") throw("'inout' expected in netlist.\n");

  // TODO: in/out nodes.
  // read_inout(in);

  in >> eat;
  if (eat != "design") throw("'design' expected in netlist.\n");

  read_design(in);

  return code_idx.size();
}

void atlsim::output(string name, std::vector<bool> &o) {
  for (unsigned i = 0; i < outputs.size(); ++i) {
    if (outputs[i].first == name) {
      outputs[i].second.v = &o;
      if (o.size() != outputs[i].second.nodes.size())
        throw("Output size mismatch.");
      return;
    }
  }

  throw("Output not found.");
}

void atlsim::input(string name, std::vector<bool> &o) {
  for (unsigned i = 0; i < inputs.size(); ++i) {
    if (inputs[i].first == name) {
      inputs[i].second.v = &o;
      if (inputs[i].second.w != o.size())
        throw("Input size mismatch.");
      return;
    }
  }

  throw("Input not found.");
}

void atlsim::get_inputs(vector<pair<string, unsigned> > &v) {
  for (unsigned i = 0; i < inputs.size(); ++i)
    v.push_back(make_pair(inputs[i].first, inputs[i].second.w));
}

void atlsim::get_outputs(vector<pair<string, unsigned> > &v) {
  for (unsigned i = 0; i < outputs.size(); ++i)
    v.push_back(make_pair(outputs[i].first, outputs[i].second.nodes.size()));
}

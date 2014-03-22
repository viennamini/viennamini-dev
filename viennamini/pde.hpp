#ifndef VIENNAMINI_PDE_HPP
#define VIENNAMINI_PDE_HPP

/* =======================================================================
   Copyright (c) 2011-2013, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the ViennaFVM base directory
======================================================================= */

#include "viennamini/forwards.h"

namespace viennamini {

struct pde
{
public:
  pde(viennamath::equation const& equ, viennamath::function_symbol const& fs) 
    : equation_(equ), function_symbol_(fs) {}

  viennamath::equation        const& equation()        { return equation_;        }
  viennamath::function_symbol const& function_symbol() { return function_symbol_; }

private:
  viennamath::equation            equation_;
  viennamath::function_symbol     function_symbol_;
};

} // viennamini

#endif


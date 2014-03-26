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

#include "viennamath/expression.hpp"

namespace viennamini {

struct pde
{
public:
  pde(viennamath::equation const& equ, viennamath::function_symbol const& fs, viennamath::expr const& damping, bool geom_update) 
    : equation_(equ), function_symbol_(fs), damping_term_(damping), geometric_update_(geom_update) {}

  viennamath::equation        const& equation()        { return equation_;        }
  viennamath::function_symbol const& function_symbol() { return function_symbol_; }
  viennamath::expr            const& damping_term()    { return damping_term_;    }
  bool                               geometric_update(){ return geometric_update_;}

private:
  viennamath::equation            equation_;
  viennamath::function_symbol     function_symbol_;
  viennamath::expr                damping_term_;
  bool                            geometric_update_;
};

} // viennamini

#endif


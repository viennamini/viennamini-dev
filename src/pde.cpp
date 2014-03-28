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

#include "viennamini/pde.hpp"

namespace viennamini {

pde::pde(viennamath::equation const& equ, viennamath::function_symbol const& fs, viennamath::expr const& damping, bool geom_update)
  : equation_(equ), function_symbol_(fs), damping_term_(damping), geometric_update_(geom_update)
{
}

viennamath::equation        const& pde::equation()
{
  return equation_;
}

viennamath::function_symbol const& pde::function_symbol()
{
  return function_symbol_;
}

viennamath::expr            const& pde::damping_term()
{
  return damping_term_;
}

bool                               pde::geometric_update()
{
  return geometric_update_;
}

} // viennamini



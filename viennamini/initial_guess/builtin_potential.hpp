#ifndef VIENNAMINI_INITIALGUESS_BUILTINPOTENTIAL_HPP
#define VIENNAMINI_INITIALGUESS_BUILTINPOTENTIAL_HPP

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

#include <vector>

#include "viennamini/forwards.h"
#include "viennamini/initial_guess.hpp"

namespace viennamini {
namespace init {

class builtin_potential : public initial_guess
{
public:
  builtin_potential() : initial_guess() 
  {
  }

  ~builtin_potential() {}

  result_type operator()(std::size_t cell_index)
  {
    return 0;
  }

private:
};

} // init
} // viennamini


#endif


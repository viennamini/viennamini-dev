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
#include "viennamini/physics.hpp"

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
    return viennamini::built_in_potential(
      get_device().get_quantity(viennamini::id::donor_doping(),     get_segment_index(), cell_index),
      get_device().get_quantity(viennamini::id::acceptor_doping(),  get_segment_index(), cell_index),
      get_device().get_quantity(viennamini::id::temperature(),      get_segment_index(), cell_index),
      get_device().get_quantity(viennamini::id::intrinsic_carrier(),get_segment_index(), cell_index)
    );
  }

private:
};

} // init
} // viennamini


#endif


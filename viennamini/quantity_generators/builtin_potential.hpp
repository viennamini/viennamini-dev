#ifndef VIENNAMINI_QUANTITYGENERATORS_BUILTINPOTENTIAL_HPP
#define VIENNAMINI_QUANTITYGENERATORS_BUILTINPOTENTIAL_HPP

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
#include "viennamini/quantity_generator.hpp"
#include "viennamini/physics.hpp"

namespace viennamini {

class builtin_potential : public quantity_generator
{
public:

  result_type operator()(std::size_t cell_index);
};

} // viennamini


#endif


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

#include "viennamini/quantity_generators/use_quantity.hpp"

namespace viennamini {

  use_quantity::use_quantity(std::string const& quantity_name) : quantity_name_(quantity_name)
  {
  }

  use_quantity::result_type use_quantity::operator()(std::size_t cell_index)
  {
    return get_device().get_quantity(quantity_name_, get_segment_index(), cell_index);
  }

} // viennamini


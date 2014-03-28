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

#include "viennamini/quantity_generators/builtin_potential.hpp"

namespace viennamini {

builtin_potential::result_type builtin_potential::operator()(std::size_t cell_index)
{
return viennamini::built_in_potential(
  get_device().get_quantity(viennamini::id::donor_doping(),     get_segment_index(), cell_index),
  get_device().get_quantity(viennamini::id::acceptor_doping(),  get_segment_index(), cell_index),
  get_device().get_quantity(viennamini::id::temperature(),      get_segment_index(), cell_index),
  get_device().get_quantity(viennamini::id::intrinsic_carrier(),get_segment_index(), cell_index)
);
}

} // viennamini


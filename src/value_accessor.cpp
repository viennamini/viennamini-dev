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

#include "viennamini/value_accessor.hpp"

namespace viennamini {

  value_accessor::value_accessor(viennamini::sparse_values& data) : data_(data)
  {

  }

  viennamini::numeric& value_accessor::operator()(std::size_t const& cell_index)
  {
    return data_[cell_index];
  }


} // viennamini


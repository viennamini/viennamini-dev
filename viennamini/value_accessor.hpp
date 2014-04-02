#ifndef VIENNAMINI_VALUEACCESSOR_HPP
#define VIENNAMINI_VALUEACCESSOR_HPP

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

class value_accessor
{
public:

  typedef viennamini::sparse_values::mapped_type result_type;

  value_accessor(viennamini::sparse_values& data);

  result_type& operator()(std::size_t const& cell_index);

private:
  viennamini::sparse_values& data_;
};

} // viennamini


#endif


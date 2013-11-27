#ifndef VIENNAMINI_UTILS_ISZERO_HPP
#define VIENNAMINI_UTILS_ISZERO_HPP

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

namespace viennamini {

template<typename NumericT>
bool is_zero(NumericT val)
{
  return (std::fabs(val) < 1.0E-20);
}

} // viennamini

#endif


/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */

#include "viennamini/quantity.hpp"

namespace viennamini
{

  quantity::quantity(viennamini::numeric value, std::string unit)
    : value_(value), unit_(unit)
  {
  }

  viennamini::numeric& quantity::value()
  {
    return value_;
  }

  std::string& quantity::unit()
  {
    return unit_;
  }
} // viennamini

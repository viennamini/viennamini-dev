#ifndef VIENNAMINI_QUANTITY_HPP
#define VIENNAMINI_QUANTITY_HPP

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

#include "viennamini/forwards.h"

namespace viennamini
{

struct quantity
{
public:
  quantity(viennamini::numeric value, std::string unit);
  quantity(viennamini::numeric value, std::string unit);


  viennamini::numeric& value();
  std::string&         unit();

private:
  viennamini::numeric value_;
  std::string         unit_;
};

} //viennamini

#endif

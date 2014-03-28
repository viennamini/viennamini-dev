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

#include "viennamini/contact_model.hpp"

namespace viennamini {

contact_model::contact_model() {}
contact_model::~contact_model() {}

void contact_model::set_quantity_name(std::string const& quantity_name)
{
  quantity_name_ = quantity_name;
}

std::string& contact_model::get_quantity_name()
{
  return quantity_name_;
}

} // viennamini


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

#include "viennamini/quantity_generator.hpp"

namespace viennamini {

quantity_generator::quantity_generator()
{
}

quantity_generator::~quantity_generator()
{
}

void quantity_generator::set_device(viennamini::device* device)
{
  device_ = device;
}

void quantity_generator::set_segment_index(std::size_t segment_index)
{
  segment_index_ = segment_index;
}

viennamini::device& quantity_generator::get_device()
{
  return *device_;
}

std::size_t&        quantity_generator::get_segment_index()
{
  return segment_index_;
}

} // viennamini



#ifndef VIENNAMINI_CONTACTMODEL_HPP
#define VIENNAMINI_CONTACTMODEL_HPP

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
#include "viennamini/device.hpp"

namespace viennamini {


class contact_model_exception : public std::runtime_error {
public:
  contact_model_exception(std::string const & str) : std::runtime_error(str) {}
};


class contact_model
{
public:

  contact_model() {}
  virtual ~contact_model() {}

  virtual void apply(viennamini::device_handle& device, std::size_t segment_index) = 0;

  void set_quantity_name(std::string const& quantity_name)  { quantity_name_ = quantity_name; }

  std::string&        get_quantity_name() { return quantity_name_; }

private:
  std::string               quantity_name_;
};

} // viennamini


#endif


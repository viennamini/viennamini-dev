#ifndef VIENNAMINI_DISCRETIZATION_HPP
#define VIENNAMINI_DISCRETIZATION_HPP

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

class required_quantity_missing : public std::runtime_error {
public:
  required_quantity_missing(std::string const & str) : std::runtime_error(str) {}
};

///brief Exception for the case that a segment could not be identified as either a contact, an oxide, nor a semiconductor
class segment_undefined_exception : public std::runtime_error {
public:
  segment_undefined_exception(int segment_index) : std::runtime_error(" at segment: "+boost::lexical_cast<std::string>(segment_index)) {}
};

class discretization
{
public:
  discretization(viennamini::device_handle        device, 
                 viennamini::configuration_handle config, 
                 viennamini::stepper_handle       stepper, 
                 std::ostream                   & stream) :
    device_(device), config_(config), stepper_(stepper), stream_(stream) {}

  virtual ~discretization() {}

  virtual void run() {}

  viennamini::device&         device()  { return *device_;  }
  viennamini::configuration&  config()  { return *config_;  }
  viennamini::stepper&        stepper() { return *stepper_; }
  std::ostream&               stream()  { return stream_;   }

private:
  viennamini::device_handle           device_;
  viennamini::configuration_handle    config_;
  viennamini::stepper_handle          stepper_;
  std::ostream                      & stream_;
};

} // viennamini


#endif


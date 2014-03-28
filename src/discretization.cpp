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

#include "viennamini/discretization.hpp"


namespace viennamini {

discretization::discretization(viennamini::device_handle        device,
               viennamini::configuration_handle config,
               viennamini::stepper_handle       stepper,
               std::ostream                   & stream) :
  device_(device), config_(config), stepper_(stepper), stream_(stream)
{
}

discretization::~discretization()
{
}

viennamini::device&         discretization::device()
{
  return *device_;
}

viennamini::device_handle&  discretization::device_handle()
{
  return  device_;
}

viennamini::configuration&  discretization::config()
{
  return *config_;
}
viennamini::stepper&        discretization::stepper()
{
  return *stepper_;
}

std::ostream&               discretization::stream()
{
  return stream_;
}

} // viennamini


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

#include "viennamini/generate_discretization.hpp"
#include "viennamini/discretizations/fvm.hpp"

namespace viennamini {

viennamini::discretization_handle generate_discretization(viennamini::discret::discret_ids   discretization_id, 
                                                          viennamini::device_handle        & device,  
                                                          viennamini::configuration_handle & config,  
                                                          viennamini::stepper_handle       & stepper, 
                                                          std::ostream                     & stream)
{
  if(discretization_id == viennamini::discret::fvm)
  {
    std::cout << "returning fvm" ;
    return discretization_handle(new viennamini::fvm(device, config, stepper, stream));
  }
  else throw discretization_exception("Discretization type is not supported by the discretization generator!");
}

} // viennamini


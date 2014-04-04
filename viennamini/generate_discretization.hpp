#ifndef VIENNAMINI_GENERATEDISCRETIZATION_HPP
#define VIENNAMINI_GENERATEDISCRETIZATION_HPP

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


namespace viennamini {

viennamini::discretization_handle generate_discretization(viennamini::discret::discret_ids   discretization_id, 
                                                          viennamini::device_handle        & device,  
                                                          viennamini::configuration_handle & config,  
                                                          viennamini::stepper_handle       & stepper, 
                                                          std::ostream                     & stream);

} // viennamini

#endif


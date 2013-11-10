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


// include necessary system headers
#include <iostream>

#include "viennamini/device_template.hpp"
#include "viennamini/device_template_capacitor2d.hpp"


int main()
{
  viennamini::device_template* device = new viennamini::capacitor2d;

  device->generate();
  
  
  delete device;  
  return 0;
}



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

#include "viennamini/templates/capacitor2d.hpp"
#include "viennamini/simulator.hpp"
//#include "viennamini/io.hpp"

int main()
{
  viennamini::device_template_handle device_generator(new viennamini::capacitor2d);
  device_generator->generate();
  viennamini::config_handle & myconfig = device_generator->config();
  viennamini::device_handle & mydevice = device_generator->device();
  mydevice->write("output");


  viennamini::simulator   mysim;
  mysim.material_library().read("../../examples/materials.xml");
  mysim.set_device(mydevice);
  mysim.set_config(myconfig);
  mysim.run();
  mysim.write("capacitor2d_result");

  return 0;
}



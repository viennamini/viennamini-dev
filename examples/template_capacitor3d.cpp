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


#include "templates/capacitor3d.hpp"
#include "viennamini/simulator.hpp"

int main()
{
  // a device generator can generate several devices, in this case a 2D Capacitor
  //
  std::string material_library_file = "../../examples/materials.xml";
  viennamini::device_template_handle device_generator(new viennamini::capacitor3d(material_library_file, std::cout));

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  device_generator->generate();
  
  // the generator produces a configuration and device object 
  //
  viennamini::config_handle & myconfig = device_generator->config();
  viennamini::device_handle & mydevice = device_generator->device();

  // setup a simulator object and link to a material file
  //
  viennamini::simulator   mysim(device_generator->stream());
  
  // set the device and config objects, generated by the above template generator
  //
  mysim.set_device(mydevice);
  mysim.set_config(myconfig);
  
  // perform the simulation
  //
  mysim.run();

  // write the simulation result to VTK files
  //
  mysim.write("capacitor3d_result");
  return 0;
}



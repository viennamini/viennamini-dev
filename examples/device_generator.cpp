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

#include "viennamini/device_template_capacitor2d.hpp"
#include "viennamini/simulator.hpp"
//#include "viennamini/io.hpp"

int main()
{
  viennamini::data_storage mystorage;

  viennamini::device_template* device_generator = new viennamini::capacitor2d(mystorage);


  viennamini::device      mydevice(mystorage);
  viennamini::config      myconfig;
  viennamini::simulator   mysim;

  mysim.set_device(mydevice);
  mysim.set_config(myconfig);

  


//  device_generator->generate();
//  
//  viennamini::device & mydevice = device_generator->device();
//  
//  viennamini::io::write_vtk(mydevice, "output");
//  
//  // prepare material library
//  viennamini::material_library mymatlib;
//  mymatlib.load("../external/ViennaMaterials/database/materials.xml");

//  // setup simulation configuration
//  viennamini::config myconfig;

//  // create a simulator object
//  viennamini::simulator   mysim(mymatlib /*, observer here? */);

//  // run the simulation
//  mysim(mydevice, myconfig);
  
  delete device_generator;  
  return 0;
}



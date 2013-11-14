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
#include "viennamini/io.hpp"

int main()
{
  viennamini::StorageType storage;
  viennamini::device_template* device_generator = new viennamini::capacitor2d(storage);

  device_generator->generate();
  
  viennamini::device & device = device_generator->device();
  
  viennamini::io::write_vtk(device, "output");
  
//  mydevice = device_generator->generate();

//  viennamini::config config;
//  config.temperature()                        = 300;
//  config.damping()                            = 1.0;
//  config.linear_breaktol()                    = 1.0E-13;
//  config.linear_iterations()                  = 700;
//  config.nonlinear_iterations()               = 100;
//  config.nonlinear_breaktol()                 = 1.0E-3;
//  config.initial_guess_smoothing_iterations() = 4;

//  viennamini::SimulatorTriangular2DType sim(device, matlib, config);
//  sim();    
  
  delete device_generator;  
  return 0;
}



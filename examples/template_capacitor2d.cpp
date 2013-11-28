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


#include "templates/capacitor2d.hpp"
#include "viennamini/simulator.hpp"

int main()
{
  viennamini::simulator  mysim(new viennamini::capacitor2d());
  mysim.device().read_material_library("../../examples/materials.xml");
  
  // (optional) get the geometry properties parameters and set some of those
  // 
  typedef viennamini::device_template::point_type PointType;
  mysim.device_generator().geometry_properties()["PI1"]  = PointType(1.0, 0.0); 
  mysim.device_generator().geometry_properties()["PI2"]  = PointType(2.0, 0.0);
  mysim.device_generator().geometry_properties()["PI3"]  = PointType(2.0, 3.0);
  mysim.device_generator().geometry_properties()["PI4"]  = PointType(1.0, 3.0);

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  mysim.device_generator().generate();
  
  // set contact potentials
  //
  mysim.current_contact_potential(1) = 1.0;
  mysim.current_contact_potential(5) = 0.0;

  // write the simulation results to output files
  //
  mysim.set_output_filename_prefix("capacitor2d_result");  
  
  // perform the simulation
  //
  mysim.run();

  return 0;
}



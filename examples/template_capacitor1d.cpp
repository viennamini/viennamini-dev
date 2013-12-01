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


#include "templates/capacitor1d.hpp"
#include "viennamini/simulator.hpp"

int main()
{
  viennamini::simulator  mysim(new viennamini::capacitor1d());
  mysim.device().read_material_library("../../examples/materials.xml");
  
  // (optional) get the geometry properties parameters and set some of those
  // 
  typedef viennamini::device_template::point_type PointType;
  mysim.device_generator().geometry_properties()["C11"]  = PointType(-0.5);
  mysim.device_generator().geometry_properties()["C1"]   = PointType(0.0);
  mysim.device_generator().geometry_properties()["I1"]   = PointType(1.0);
  mysim.device_generator().geometry_properties()["I2"]   = PointType(2.0);
  mysim.device_generator().geometry_properties()["C2"]   = PointType(3.0);
  mysim.device_generator().geometry_properties()["C21"]  = PointType(3.5);

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  mysim.device_generator().generate();

  // set contact potentials
  //
  mysim.contact_potential(1) = 1.0;
  mysim.contact_potential(5) = 0.0;

  // write the simulation results to output files
  //
  mysim.set_output_filename_prefix("capacitor1d_result");  
  
  // perform the simulation
  //
  mysim.run();

  return 0;
}



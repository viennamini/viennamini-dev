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
  viennamini::simulator  mysim(new viennamini::capacitor3d());
  mysim.device().read_material_library("../../examples/materials.xml");

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  mysim.device_generator().generate();
  
  // set contact potentials
  //
  mysim.current_contact_potential(1) = 1.0;
  mysim.current_contact_potential(5) = 0.0;

  // write the simulation results to output files
  //
  mysim.set_output_filename_prefix("capacitor3d_result");  
  
  // perform the simulation
  //
  mysim.run();
  return 0;
}



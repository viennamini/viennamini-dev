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


#include "templates/diode_np2d.hpp"
#include "viennamini/simulator.hpp"

int main()
{
  viennamini::simulator  mysim(new viennamini::diode_np2d());
  mysim.device().read_material_library("../../examples/materials.xml");

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  mysim.device_generator().generate();
  
  // set contact potentials
  //
  int anode_id = mysim.device_generator().segment_indices()["Anode"];
  mysim.set_contact_potential_range(anode_id, -1.0, 1.0, 0.05);

  // write the simulation results to output files
  //
  mysim.set_output_filename_prefix("diode_np2d_result");  
  
  // perform the simulation
  //
  mysim.run();

  viennamini::data_table table = mysim.data_table();
  table.write("diode_np2d_characteristics.csv");

  return 0;
}



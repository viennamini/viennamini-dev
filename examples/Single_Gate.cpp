/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Resutik Peter         e1126613@student.tuwien.ac.at

   license:    see file LICENSE in the base directory
======================================================================= */

// ViennaMini includes
//
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"


int main()
{
  // create the simulator object
  //
  viennamini::simulator  mysim(std::cout);

  // read mesh and material input files
  //
  mysim.device().read(viennamini::device_collection_path()+"/Single_Gate/Single_Gate/Single_Gate_meshed.pvd", viennamini::tetrahedral_3d());
  mysim.device().read_material_database("../../auxiliary/materials.xml");
  mysim.device().read_unit_database("../../auxiliary/units.xml");

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
  mysim.device().scale(1.0E-9);

 // set the temperature of the device
  //
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0, "K");

  // setup auxiliary segment indices, aiding in identifying the individual
  // device segments in the subsequent device setup step
  //
  const int silicon         = 1;
  const int channel         = 2;
  const int oxide	    = 3;
  const int source          = 4;
  const int drain           = 5;
  const int source_contact  = 6;
  const int drain_contact   = 7;
  const int bulk_contact    = 8;
  const int gate_contact    = 9;

// setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::semiconductor,  source,         "source",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  silicon,        "silicon",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  channel,        "channel",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain,          "drain",           "Si");
//  std::cout << "55 Number of mesh-segments: " << mysim.device().get_segmesh_tetrahedral_3d().segmentation.size() << std::endl;
  mysim.device().make(viennamini::role::oxide,          oxide,          "oxide",           "HfO2");
//  std::cout << "66 Number of mesh-segments: " << mysim.device().get_segmesh_tetrahedral_3d().segmentation.size() << std::endl;
  mysim.device().make(viennamini::role::contact,        gate_contact,   "gate_contact",    "Cu");
  mysim.device().make(viennamini::role::contact,        source_contact, "source_contact",  "Cu");
  mysim.device().make(viennamini::role::contact,        drain_contact,  "drain_contact",   "Cu");
  mysim.device().make(viennamini::role::contact,        bulk_contact,   "bulk_contact",    "Cu");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(viennamini::id::donor_doping(),    source,     1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source,     1.0E8,  "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain,      1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain,      1.0E8,  "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    channel,    1.0E22, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), channel,    1.0E10, "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    silicon,    1.0E22, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), silicon,    1.0E10, "m-3");

// set optional solver parameters
  //
  mysim.config().linear_breaktol()                    = 1.0E-10;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-2;
  mysim.config().damping()                            = 0.6; //0.9

// set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact_quantity(viennamini::id::potential(), gate_contact,   0.6, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), source_contact, 0.0, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), drain_contact,  0.2, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), bulk_contact,   0.0, "V");


  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}

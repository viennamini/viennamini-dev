/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

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
  mysim.device().read(viennamini::device_collection_path()+"/mosfet_utb_soi_3d/mosfet_utb_soi_3d_main.pvd", viennamini::tetrahedral_3d());
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
  const int body            = 1;
  const int body_contact    = 2;
  const int buried_oxide    = 3;
  const int channel         = 4;
  const int source          = 5;
  const int source_contact  = 6;
  const int drain           = 7;
  const int drain_contact   = 8;
  const int gate_oxide      = 9;
  const int gate_contact    = 10;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::semiconductor,  source,         "source",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  channel,        "channel",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain,          "drain",           "Si");
  mysim.device().make(viennamini::role::oxide,          gate_oxide,     "gate_oxide",      "HfO2");
  mysim.device().make(viennamini::role::oxide,          buried_oxide,   "buried_oxide",    "SiO2");
  mysim.device().make(viennamini::role::contact,        gate_contact,   "gate_contact",    "Cu");
  mysim.device().make(viennamini::role::semiconductor,  body,           "body",            "Si");
  mysim.device().make(viennamini::role::contact,        body_contact,   "body_contact",    "Cu");
  mysim.device().make(viennamini::role::contact,        source_contact, "source_contact",  "Cu");
  mysim.device().make(viennamini::role::contact,        drain_contact,  "drain_contact",   "Cu");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(viennamini::id::donor_doping(),    channel, 1.0E22, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), channel, 1.0E10, "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    source,  1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source,  1.0E8,  "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain,   1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain,   1.0E8,  "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    body,    1.0E22, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), body,    1.0E10, "m-3");

  // set optional solver parameters
  //
  mysim.config().linear_solver().breaktol()        = 1.0E-14;
  mysim.config().linear_solver().iterations()      = 1000;
  mysim.config().nonlinear_solver().iterations()   = 100;
  mysim.config().nonlinear_solver().breaktol()     = 1.0E-2;
  mysim.config().nonlinear_solver().damping()      = 0.9;

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().pdeset_id()         = viennamini::pdeset::drift_diffusion;
  mysim.config().model().discretization_id() = viennamini::discret::fvm;

  // manually set the contact potentials
  //
  mysim.device().set_contact_quantity(viennamini::id::potential(), gate_contact,   0.6, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), source_contact, 0.0, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), drain_contact,  0.2, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), body_contact,   0.0, "V");

  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}


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

// ViennaMini includes
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"

int main()
{
  // create the simulator object
  //
  viennamini::simulator  mysim(std::cout);

  // read mesh and material input files
  //
  mysim.device().read(viennamini::device_collection_path()+"/mosfet2d/mosfet2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_library("../../examples/materials.xml");

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
  mysim.device().scale(1.0E-9);

  // set the temperature of the device
  //
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0);

  // setup auxiliary segment indices, aiding in identifying the individual
  // device segments in the subsequent device setup step
  //
  const int gate_contact    = 1;
  const int source_contact  = 2;
  const int oxide           = 3;
  const int drain_contact   = 4;
  const int source          = 5;
  const int drain           = 6;
  const int body            = 7;
  const int body_contact    = 8;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::contact,        gate_contact,   "gate_contact",   "Cu");
  mysim.device().make(viennamini::role::contact,        source_contact, "source_contact", "Cu");
  mysim.device().make(viennamini::role::contact,        body_contact,   "body_contact",   "Cu");
  mysim.device().make(viennamini::role::contact,        drain_contact,  "drain_contact",  "Cu");
  mysim.device().make(viennamini::role::oxide,          oxide,          "oxide",          "HfO2");
  mysim.device().make(viennamini::role::semiconductor,  source,         "source",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain,          "drain",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  body,           "body",           "Si");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(viennamini::id::donor_doping(),    source, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain,  1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain,  1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    body,   1.0E22);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), body,   1.0E10);

//  mysim.device().set_quantity(viennamini::id::potential(), oxide,             0.0);
//  mysim.device().set_quantity(viennamini::id::acceptor_doping(),       1.0);
//  mysim.device().set_quantity(viennamini::id::donor_doping(),          1.0);
//  mysim.device().set_quantity(viennamini::id::electron_mobility(),     1.0);
//  mysim.device().set_quantity(viennamini::id::hole_mobility(),         1.0);
//  mysim.device().set_quantity(viennamini::id::intrinsic_carrier(),     1.0);
//  mysim.device().set_quantity(viennamini::id::temperature(),           300.0);
//  mysim.device().set_quantity(viennamini::id::thermal_potential(),     0.25);






  // set optional solver parameters
  //
  mysim.config().linear_breaktol()        = 1.0E-14;
  mysim.config().linear_iterations()      = 1000;
  mysim.config().nonlinear_iterations()   = 1;
  mysim.config().nonlinear_breaktol()     = 1.0E-2;
  mysim.config().damping()                = 0.9;

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact(viennamini::id::potential(), gate_contact,   0.0);
  mysim.device().set_contact(viennamini::id::potential(), source_contact, 0.0);
  mysim.device().set_contact(viennamini::id::potential(), drain_contact,  0.0);
  mysim.device().set_contact(viennamini::id::potential(), body_contact,   0.0);

  // perform the simulation
  //
  mysim.run();

  std::cout << "**********************************************************" << std::endl;
  std::cout << "* MOSFET 2D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "**********************************************************" << std::endl;

  return EXIT_SUCCESS;
}


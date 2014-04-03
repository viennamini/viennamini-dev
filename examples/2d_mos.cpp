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
  viennamini::simulator  mysim;

  // read mesh and material input files
  //
  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
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
  const int left_contact      = 1;
  const int oxide             = 2;
  const int semiconductor_one = 3;
  const int semiconductor_two = 4;
  const int right_contact     = 5;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::contact,        left_contact,       "left_contact",     "Cu");
  mysim.device().make(viennamini::role::oxide,          oxide,              "oxide",            "HfO2");
  mysim.device().make(viennamini::role::semiconductor,  semiconductor_one,  "semiconductor_one","Si");
  mysim.device().make(viennamini::role::semiconductor,  semiconductor_two,  "semiconductor_two","Si");
  mysim.device().make(viennamini::role::contact,        right_contact,      "right_contact",    "Cu");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(viennamini::id::donor_doping(),    semiconductor_one, 1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), semiconductor_one, 1.0E8,  "m-3");
  mysim.device().set_quantity(viennamini::id::donor_doping(),    semiconductor_two, 1.0E24, "m-3");
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), semiconductor_two, 1.0E8,  "m-3");

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // set optional solver parameters
  //
  mysim.config().linear_breaktol()        = 1.0E-14;
  mysim.config().linear_iterations()      = 1000;
  mysim.config().nonlinear_iterations()   = 100;
  mysim.config().nonlinear_breaktol()     = 1.0E-2;
  mysim.config().damping()                = 0.9;

  // manually set the contact potentials
  //
  mysim.device().set_contact_quantity(viennamini::id::potential(), left_contact,  0.0, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), right_contact, 0.0, "V");

  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}


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
  using namespace viennamini;

  // create the simulator object
  //
  simulator  mysim;

  // read mesh and material input files
  //
  mysim.device().read(device_collection_path()+"/nin2d/nin2d.mesh", triangular_2d());
  mysim.device().read_material_library("../../auxiliary/materials.xml");

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
  mysim.device().scale(1.0E-9);

  // set the temperature of the device
  //
  mysim.device().set_quantity(id::temperature(), 300.0, unit::si::kelvin());

  // setup auxiliary segment indices, aiding in identifying the individual
  // device segments in the subsequent device setup step
  //
  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(role::contact,        left_contact,  "left_contact",  "Cu");
  mysim.device().make(role::semiconductor,  left,          "left",          "Si");
  mysim.device().make(role::semiconductor,  intrinsic,     "intrinsic",     "Si");
  mysim.device().make(role::semiconductor,  right,         "right",        "Si");
  mysim.device().make(role::contact,        right_contact, "right_contact", "Cu");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(id::donor_doping(),    left,      1.0E24, unit::si::carrier_concentration());
  mysim.device().set_quantity(id::acceptor_doping(), left,      1.0E8,  unit::si::carrier_concentration());
  mysim.device().set_quantity(id::donor_doping(),    intrinsic, 1.0E21, unit::si::carrier_concentration());
  mysim.device().set_quantity(id::acceptor_doping(), intrinsic, 1.0E11, unit::si::carrier_concentration());
  mysim.device().set_quantity(id::donor_doping(),    right,     1.0E24, unit::si::carrier_concentration());
  mysim.device().set_quantity(id::acceptor_doping(), right,     1.0E8,  unit::si::carrier_concentration());

  // set optional solver parameters
  //
  mysim.config().linear_breaktol()      = 1.0E-14;
  mysim.config().linear_iterations()    = 1000;
  mysim.config().nonlinear_iterations() = 100;
  mysim.config().nonlinear_breaktol()   = 1.0E-3;

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(pdeset::drift_diffusion);
  mysim.config().model().set_discretization(discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact(id::potential(), left_contact,  0.0);
  mysim.device().set_contact(id::potential(), right_contact, 0.2);


  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}


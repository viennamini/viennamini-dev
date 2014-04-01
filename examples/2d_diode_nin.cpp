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
  mysim.device().read_material_library("../../examples/materials.xml");

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
  mysim.device().scale(1.0E-9);

  // set the temperature of the device
  //
  // typedef viennamini::value         data;
  // typedef viennamini::dense_values  data;
  // typedef viennamini::sparse_values data;
  // mysim.device().set_quantity(viennamini::id::temperature(), viennamini::quantity(data,  viennamini::unit::K));
  // mysim.device().set_quantity(viennamini::id::temperature(), viennamini::quantity(300.0, viennamini::unit::K));
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0);

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
  mysim.device().make(viennamini::role::contact,        left_contact,  "left_contact",  "Cu");
  mysim.device().make(viennamini::role::semiconductor,  left,          "left",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  intrinsic,     "intrinsic",     "Si");
  mysim.device().make(viennamini::role::semiconductor,  right,         "right",        "Si");
  mysim.device().make(viennamini::role::contact,        right_contact, "right_contact", "Cu");

  // assign doping values to the semiconductor segments
  //
  mysim.device().set_quantity(viennamini::id::donor_doping(),    left, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), left, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    intrinsic, 1.0E21);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), intrinsic, 1.0E11);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    right, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), right, 1.0E8);

  // set optional solver parameters
  //
  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact(viennamini::id::potential(), left_contact,  0.0);
  mysim.device().set_contact(viennamini::id::potential(), right_contact, 0.2);


  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}


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
  viennamini::simulator  mysim;

  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0);

  // identify segments
  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  mysim.device().make(viennamini::role::contact,        left_contact,  "left_contact",  "Cu");
  mysim.device().make(viennamini::role::semiconductor,  left,          "left",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  intrinsic,     "intrinsic",     "Si");
  mysim.device().make(viennamini::role::semiconductor,  right,         "right",        "Si");
  mysim.device().make(viennamini::role::contact,        right_contact, "right_contact", "Cu");

  mysim.device().set_quantity(viennamini::id::donor_doping(),    left, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), left, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    intrinsic, 1.0E21);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), intrinsic, 1.0E11);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    right, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), right, 1.0E8);

  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;

  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  mysim.device().set_contact(viennamini::id::potential(), left_contact,  0.0);
  mysim.device().set_contact(viennamini::id::potential(), right_contact, 0.2);



  mysim.run();

  return EXIT_SUCCESS;
}


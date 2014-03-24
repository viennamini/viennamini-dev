/* =======================================================================
   Copyright (c) 2011-2013, Institute for Microelectronics, TU Wien
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
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"


int main()
{
  viennamini::simulator  mysim;

  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);
  mysim.device().temperature() = 300;

  // identify segments
  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  mysim.device().make_contact         (left_contact);
  mysim.device().set_name             (left_contact, "left_contact");
  mysim.device().set_material         (left_contact, "Cu");

  mysim.device().make_semiconductor   (left);
  mysim.device().set_name             (left, "left");
  mysim.device().set_material         (left, "Si");

  mysim.device().make_semiconductor   (intrinsic);
  mysim.device().set_name             (intrinsic, "left");
  mysim.device().set_material         (intrinsic, "Si");

  mysim.device().make_semiconductor   (right);
  mysim.device().set_name             (right, "right");
  mysim.device().set_material         (right, "Si");

  mysim.device().make_contact         (right_contact);
  mysim.device().set_name             (right_contact, "right_contact");
  mysim.device().set_material         (right_contact, "Cu");

  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().write_initial_guess_files()          = true;
  mysim.config().write_result_files()                 = true;

  mysim.device().set_quantity(viennamini::id::potential(), left_contact,  0.0);
  mysim.device().set_quantity(viennamini::id::potential(), right_contact, 0.2);

  mysim.config().model().use_pdeset(viennamini::pdeset::laplace);
  mysim.config().model().use_discretization(viennamini::discret::fvm);

  mysim.run();

  return EXIT_SUCCESS;
}


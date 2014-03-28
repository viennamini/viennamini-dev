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

int main(int argc, char* argv[])
{
  viennamini::simulator  mysim;

  mysim.device().read(viennamini::device_collection_path()+"/half-trigate3d/half-trigate3d.mesh", viennamini::tetrahedral_3d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0);

  const int source          = 1;
  const int channel         = 2;
  const int drain           = 3;
  const int oxide           = 4;
  const int gate_contact    = 5;
  const int body            = 6;
  const int body_contact    = 7;
  const int source_contact  = 8;
  const int drain_contact   = 9;

  mysim.device().make(viennamini::role::semiconductor,  source,         "source",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  channel,        "channel",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain,          "drain",           "Si");
  mysim.device().make(viennamini::role::oxide,          oxide,          "oxide",           "HfO2");
  mysim.device().make(viennamini::role::contact,        gate_contact,   "gate_contact",    "Cu");
  mysim.device().make(viennamini::role::semiconductor,  body,           "body",            "Si");
  mysim.device().make(viennamini::role::contact,        body_contact,   "body_contact",    "Cu");
  mysim.device().make(viennamini::role::contact,        source_contact, "source_contact",  "Cu");
  mysim.device().make(viennamini::role::contact,        drain_contact,  "drain_contact",   "Cu");

  mysim.device().set_quantity(viennamini::id::donor_doping(),    source, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    channel, 1.0E12);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), channel, 1.0E20);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    body, 1.0E12);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), body, 1.0E20);


  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-2;
  mysim.config().damping()                            = 0.5;
  mysim.config().write_initial_guess_files()          = false;
  mysim.config().write_result_files()                 = true;

  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact(viennamini::id::potential(), gate_contact,   0.2);
  mysim.device().set_contact(viennamini::id::potential(), source_contact, 0.0);
  mysim.device().set_contact(viennamini::id::potential(), drain_contact,  0.2);
  mysim.device().set_contact(viennamini::id::potential(), body_contact,   0.0);

  mysim.run();

  std::cout << "***********************************************************" << std::endl;
  std::cout << "* TRIGATE 3D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "***********************************************************" << std::endl;
  return EXIT_SUCCESS;
}


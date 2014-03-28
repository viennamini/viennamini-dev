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
  viennamini::simulator  mysim(std::cout);

  mysim.device().read(viennamini::device_collection_path()+"/mosfet_planar_3d/mosfet_planar_3d_main.pvd", viennamini::tetrahedral_3d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);
  mysim.device().set_quantity(viennamini::id::temperature(), 300.0);

  const int body            = 1;
  const int body_contact    = 2;
  const int channel         = 3;
  const int source          = 4;
  const int source_ext      = 5;
  const int source_contact  = 6;
  const int drain           = 7;
  const int drain_ext       = 8;
  const int drain_contact   = 9;
  const int oxide           = 10;
  const int gate_contact    = 11;

  mysim.device().make(viennamini::role::semiconductor,  source,         "source",          "Si");
  mysim.device().make(viennamini::role::semiconductor,  source_ext,     "source_ext",      "Si");
  mysim.device().make(viennamini::role::semiconductor,  channel,        "channel",         "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain,          "drain",           "Si");
  mysim.device().make(viennamini::role::semiconductor,  drain_ext,      "drain_ext",       "Si");
  mysim.device().make(viennamini::role::oxide,          oxide,          "oxide",           "HfO2");
  mysim.device().make(viennamini::role::contact,        gate_contact,   "gate_contact",    "Cu");
  mysim.device().make(viennamini::role::semiconductor,  body,           "body",            "Si");
  mysim.device().make(viennamini::role::contact,        body_contact,   "body_contact",    "Cu");
  mysim.device().make(viennamini::role::contact,        source_contact, "source_contact",  "Cu");
  mysim.device().make(viennamini::role::contact,        drain_contact,  "drain_contact",   "Cu");

  mysim.device().set_quantity(viennamini::id::donor_doping(),    source, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    channel, 1.0E22);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), channel, 1.0E10);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    source_ext, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), source_ext, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    drain_ext, 1.0E24);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), drain_ext, 1.0E8);
  mysim.device().set_quantity(viennamini::id::donor_doping(),    body, 1.0E22);
  mysim.device().set_quantity(viennamini::id::acceptor_doping(), body, 1.0E10);

  mysim.config().linear_breaktol()                    = 1.0E-10;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-2;
  mysim.config().damping()                            = 0.9;

  mysim.config().model().set_pdeset(viennamini::pdeset::drift_diffusion);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact(viennamini::id::potential(), gate_contact,   0.2);
  mysim.device().set_contact(viennamini::id::potential(), source_contact, 0.0);
  mysim.device().set_contact(viennamini::id::potential(), drain_contact,  0.2);
  mysim.device().set_contact(viennamini::id::potential(), body_contact,   0.0);

  mysim.run();

  std::cout << "*****************************************************************" << std::endl;
  std::cout << "* MOSFET PLANAR 3D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "*****************************************************************" << std::endl;

  return EXIT_SUCCESS;
}


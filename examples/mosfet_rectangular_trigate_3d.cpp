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

  mysim.device().read(viennamini::device_collection_path()+"/mosfet_rectangular_trigate_3d/mosfet_rectangular_trigate_3d_main.pvd", viennamini::tetrahedral_3d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);
  mysim.device().temperature() = 300;

  const int body            = 1;
  const int body_contact    = 2;
  const int source          = 3;
  const int source_contact  = 4;
  const int channel         = 5;
  const int drain           = 6;
  const int drain_contact   = 7;
  const int inner_oxide     = 8;
  const int outer_oxide     = 9;
  const int gate_contact    = 10;

  mysim.device().make_contact             (gate_contact);
  mysim.device().set_name                 (gate_contact, "gate_contact");
  mysim.device().set_material             (gate_contact, "Cu");

  mysim.device().make_contact           (source_contact);
  mysim.device().set_name               (source_contact, "source_contact");
  mysim.device().set_material           (source_contact, "Cu");

  mysim.device().make_oxide             (inner_oxide);
  mysim.device().set_name               (inner_oxide, "oxide");
  mysim.device().set_material           (inner_oxide, "SiO2");

  mysim.device().make_oxide             (outer_oxide);
  mysim.device().set_name               (outer_oxide, "oxide");
  mysim.device().set_material           (outer_oxide, "HfO2");

  mysim.device().make_contact           (drain_contact);
  mysim.device().set_name               (drain_contact, "drain_contact");
  mysim.device().set_material           (drain_contact, "Cu");

  mysim.device().make_semiconductor     (source);
  mysim.device().set_name               (source, "source");
  mysim.device().set_material           (source, "Si");
  mysim.device().set_donator_doping     (source, 1.0E24);
  mysim.device().set_acceptor_doping    (source, 1.0E8);

  mysim.device().make_semiconductor     (drain);
  mysim.device().set_name               (drain, "drain");
  mysim.device().set_material           (drain, "Si");
  mysim.device().set_donator_doping     (drain, 1.0E24);
  mysim.device().set_acceptor_doping    (drain, 1.0E8);

  mysim.device().make_semiconductor     (channel);
  mysim.device().set_name               (channel, "channel");
  mysim.device().set_material           (channel, "Si");
  mysim.device().set_donator_doping     (channel, 1.0E22);
  mysim.device().set_acceptor_doping    (channel, 1.0E10);

  mysim.device().make_semiconductor     (body);
  mysim.device().set_name               (body, "body");
  mysim.device().set_material           (body, "Si");
  mysim.device().set_donator_doping     (body, 1.0E22);
  mysim.device().set_acceptor_doping    (body, 1.0E10);

  mysim.device().make_contact           (body_contact);
  mysim.device().set_name               (body_contact, "body_contact");
  mysim.device().set_material           (body_contact, "Cu");

  mysim.config().linear_breaktol()                    = 1.0E-10;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-2;
  mysim.config().damping()                            = 0.9;
  mysim.config().write_initial_guess_files()          = true;
  mysim.config().write_result_files()                 = true;

  mysim.problem_id() = viennamini::id::poisson_drift_diffusion_np();

  // manually set the contact potentials
  //
  mysim.contact_workfunction(gate_contact)   = 0.4;
  mysim.contact_potential   (gate_contact)   = 0.4;
  mysim.contact_potential   (source_contact) = 0.0;
  mysim.contact_potential   (drain_contact)  = 0.5;
  mysim.contact_potential   (body_contact)   = 0.0;

  mysim.set_output_filename_prefix("mosfet_rectangular_trigate_3d_result");

  mysim.run();

  std::cout << "******************************************************************************" << std::endl;
  std::cout << "* MOSFET RECTANGULAR TRIGATE 3D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "******************************************************************************" << std::endl;

  return EXIT_SUCCESS;
}


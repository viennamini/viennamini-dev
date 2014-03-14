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
  mysim.device().temperature() = 300;

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

  mysim.device().make_contact             (gate_contact);
  mysim.device().set_name                 (gate_contact, "gate_contact");
  mysim.device().set_material             (gate_contact, "Cu");

  mysim.device().make_contact           (source_contact);
  mysim.device().set_name               (source_contact, "source_contact");
  mysim.device().set_material           (source_contact, "Cu");

  mysim.device().make_oxide             (oxide);
  mysim.device().set_name               (oxide, "oxide");
  mysim.device().set_material           (oxide, "HfO2");

  mysim.device().make_contact           (drain_contact);
  mysim.device().set_name               (drain_contact, "drain_contact");
  mysim.device().set_material           (drain_contact, "Cu");

  mysim.device().make_semiconductor     (source);
  mysim.device().set_name               (source, "source");
  mysim.device().set_material           (source, "Si");
  mysim.device().set_donator_doping     (source, 1.0E24);
  mysim.device().set_acceptor_doping    (source, 1.0E8);

  mysim.device().make_semiconductor     (source_ext);
  mysim.device().set_name               (source_ext, "source_extension");
  mysim.device().set_material           (source_ext, "Si");
  mysim.device().set_donator_doping     (source_ext, 1.0E24);
  mysim.device().set_acceptor_doping    (source_ext, 1.0E8);

  mysim.device().make_semiconductor     (drain);
  mysim.device().set_name               (drain, "drain");
  mysim.device().set_material           (drain, "Si");
  mysim.device().set_donator_doping     (drain, 1.0E24);
  mysim.device().set_acceptor_doping    (drain, 1.0E8);

  mysim.device().make_semiconductor     (drain_ext);
  mysim.device().set_name               (drain_ext, "drain_extension");
  mysim.device().set_material           (drain_ext, "Si");
  mysim.device().set_donator_doping     (drain_ext, 1.0E24);
  mysim.device().set_acceptor_doping    (drain_ext, 1.0E8);

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

  mysim.set_output_filename_prefix("mosfet_planar_3d_result");

  mysim.run();

  std::cout << "*****************************************************************" << std::endl;
  std::cout << "* MOSFET PLANAR 3D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "*****************************************************************" << std::endl;

  return EXIT_SUCCESS;
}


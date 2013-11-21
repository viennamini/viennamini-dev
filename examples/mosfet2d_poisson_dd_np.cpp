/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
           ViennaFVM - The Vienna Finite Volume Method Library
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    To be discussed, see file LICENSE in the ViennaFVM base directory
======================================================================= */

// ViennaMini includes
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"

int main()
{
  viennamini::simulator  mysim;

  mysim.device().read(viennamini::device_collection_path()+"/mosfet2d/mosfet2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);

  const int gate_contact    = 1;
  const int source_contact  = 2;
  const int oxide           = 3;
  const int drain_contact   = 4;
  const int source          = 5;
  const int drain           = 6;
  const int body            = 7;
  const int body_contact    = 8;

  mysim.device().make_contact             (gate_contact);
  mysim.device().set_name                 (gate_contact, "gate_contact");
  mysim.device().set_material             (gate_contact, "Cu");
  mysim.device().set_contact_potential    (gate_contact, 0.2);
  mysim.device().add_contact_workfunction (gate_contact, 0.4);

  mysim.device().make_contact           (source_contact);
  mysim.device().set_name               (source_contact, "source_contact");
  mysim.device().set_material           (source_contact, "Cu");
  mysim.device().set_contact_potential  (source_contact, 0.0);

  mysim.device().make_oxide             (oxide);
  mysim.device().set_name               (oxide, "oxide");
  mysim.device().set_material           (oxide, "HfO2");

  mysim.device().make_contact           (drain_contact);
  mysim.device().set_name               (drain_contact, "drain_contact");
  mysim.device().set_material           (drain_contact, "Cu");
  mysim.device().set_contact_potential  (drain_contact, 0.2);

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

  mysim.device().make_semiconductor     (body);
  mysim.device().set_name               (body, "body");
  mysim.device().set_material           (body, "Si");
  mysim.device().set_donator_doping     (body, 1.0E12);
  mysim.device().set_acceptor_doping    (body, 1.0E20);

  mysim.device().make_contact           (body_contact);
  mysim.device().set_name               (body_contact, "body_contact");
  mysim.device().set_material           (body_contact, "Cu");
  mysim.device().set_contact_potential  (body_contact, 0.0);

  mysim.config().temperature()                        = 300;
  mysim.config().damping()                            = 1.0;
  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;
  mysim.config().initial_guess_smoothing_iterations() = 4;
  mysim.config().problem()                            = viennamini::id::poisson_drift_diffusion_np();

  mysim.run();

  mysim.write("mosfet2d_dd_np_result");

  std::cout << "**********************************************************" << std::endl;
  std::cout << "* MOSFET 2D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "**********************************************************" << std::endl;

  return EXIT_SUCCESS;
}


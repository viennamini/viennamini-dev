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
  mysim.device().set_donator_doping   (left, 1.0E24);
  mysim.device().set_acceptor_doping  (left, 1.0E8);
//  mysim.device().set_mobility         (left, viennamini::mobility::lattice);
//  mysim.device().set_recombination    (left, viennamini::recombination::srh);

  mysim.device().make_semiconductor   (intrinsic);
  mysim.device().set_name             (intrinsic, "left");
  mysim.device().set_material         (intrinsic, "Si");
  mysim.device().set_donator_doping   (intrinsic, 1.0E21);
  mysim.device().set_acceptor_doping  (intrinsic, 1.0E11);
//  mysim.device().set_mobility         (intrinsic, viennamini::mobility::lattice);
//  mysim.device().set_recombination    (intrinsic, viennamini::recombination::srh);

  mysim.device().make_semiconductor   (right);
  mysim.device().set_name             (right, "right");
  mysim.device().set_material         (right, "Si");
  mysim.device().set_donator_doping   (right, 1.0E24);
  mysim.device().set_acceptor_doping  (right, 1.0E8);
//  mysim.device().set_mobility         (right, viennamini::mobility::lattice);
//  mysim.device().set_recombination    (right, viennamini::recombination::srh);

  mysim.device().make_contact         (right_contact);
  mysim.device().set_name             (right_contact, "right_contact");
  mysim.device().set_material         (right_contact, "Cu");

  mysim.config().temperature()                        = 300;
  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;
  mysim.config().initial_guess_smoothing_iterations() = 4;
  mysim.config().problem()                            = viennamini::id::poisson_drift_diffusion_np();
  mysim.config().write_initial_guess_files()          = true;
  mysim.config().write_result_files()                 = true;

  mysim.current_contact_potential   (left_contact)   = 0.0;
  mysim.current_contact_potential   (right_contact) = 0.2;

  mysim.set_output_filename_prefix("nin2d_dd_np_result");

  mysim.run();

  std::cout << "*******************************************************" << std::endl;
  std::cout << "* NIN 2D DD Bipolar simulation finished successfully! *" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  return EXIT_SUCCESS;
}


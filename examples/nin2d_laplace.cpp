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

  mysim.material_library().read("../../examples/materials.xml");

  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().scale(1.0E-9);

  // identify segments
  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  mysim.device().make_contact(left_contact);
  mysim.device().name(left_contact)     = "left_contact";
  mysim.device().material(left_contact) = "Cu";
  mysim.device().contact_potential(left_contact) = 0.0;
  mysim.device().workfunction(left_contact) = 0.0;

  mysim.device().make_semiconductor(left);
  mysim.device().name(left)     = "left";
  mysim.device().material(left) = "Si";

  mysim.device().make_semiconductor(intrinsic);
  mysim.device().name(intrinsic)     = "left";
  mysim.device().material(intrinsic) = "Si";

  mysim.device().make_semiconductor(right);
  mysim.device().name(right)     = "right";
  mysim.device().material(right) = "Si";

  mysim.device().make_contact(right_contact);
  mysim.device().name(right_contact)     = "right_contact";
  mysim.device().material(right_contact) = "Cu";
  mysim.device().contact_potential(right_contact) = 0.2;
  mysim.device().workfunction(right_contact) = 0.0;

  mysim.config().temperature()                        = 300;
  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().problem()                            = viennamini::id::laplace();

  mysim.run();
  
  mysim.write("nin2d_laplace_result");

  std::cout << "****************************************************" << std::endl;
  std::cout << "* NIN 2D Laplace simulation finished successfully! *" << std::endl;
  std::cout << "****************************************************" << std::endl;
  return EXIT_SUCCESS;
}


/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
           ViennaFVM - The Vienna Finite Volume Method Library
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               (add your name here)

   license:    To be discussed, see file LICENSE in the ViennaFVM base directory
======================================================================= */


// ViennaMini includes
#include "viennamini/simulator.hpp"
#include "viennamini/scale.hpp"
#include "viennamini/io.hpp"


int main()
{
  viennamini::storage   mystorage;
  viennamini::device    mydevice(mystorage);

  // import a VTK file
  viennamini::io::read_vtk(mydevice, "../external/ViennaDeviceCollection/nin2d/nin2d.mesh", viennagrid::config::triangular_2d());

  // scale the mesh to the nanometer regime
  viennamini::scale(mydevice, 1.0E-9);

  // identify segments
  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  mydevice.make_contact(left_contact);
  mydevice.name(left_contact)     = "left_contact";
  mydevice.material(left_contact) = "Cu";
  mydevice.contact_potential(left_contact) = 0.0;
  mydevice.workfunction(left_contact) = 0.0;

  mydevice.make_semiconductor(left);
  mydevice.name(left)     = "left";
  mydevice.material(left) = "Si";
  mydevice.ND_max(left) = 1.0E24;
  mydevice.NA_max(left) = 1.0E8;

  mydevice.make_semiconductor(intrinsic);
  mydevice.name(intrinsic)     = "left";
  mydevice.material(intrinsic) = "Si";
  mydevice.ND_max(intrinsic) = 1.0E21;
  mydevice.NA_max(intrinsic) = 1.0E11;

  mydevice.make_semiconductor(right);
  mydevice.name(right)     = "right";
  mydevice.material(right) = "Si";
  mydevice.ND_max(right) = 1.0E24;
  mydevice.NA_max(right) = 1.0E8;

  mydevice.make_contact(right_contact);
  mydevice.name(right_contact)     = "right_contact";
  mydevice.material(right_contact) = "Cu";
  mydevice.contact_potential(right_contact) = 0.5;
  mydevice.workfunction(right_contact) = 0.0;

  // create a device and a config object
  viennamini::config myconfig;
  myconfig.temperature()                        = 300;
  myconfig.damping()                            = 1.0;
  myconfig.linear_breaktol()                    = 1.0E-13;
  myconfig.linear_iterations()                  = 700;
  myconfig.nonlinear_iterations()               = 100;
  myconfig.nonlinear_breaktol()                 = 1.0E-3;
  myconfig.initial_guess_smoothing_iterations() = 4;

  // prepare material library
  viennamini::material_library mymatlib;
  mymatlib.load("/home/weinbub/git/ViennaMini/external/ViennaMaterials/database/materials.xml");

  // create a simulator object
  viennamini::simulator   mysim(mymatlib);

  // run the simulation
  mysim(mydevice, myconfig);

  std::cout << "********************************************" << std::endl;
  std::cout << "* NIN2D simulation finished successfully! *" << std::endl;
  std::cout << "********************************************" << std::endl;
  return EXIT_SUCCESS;
}


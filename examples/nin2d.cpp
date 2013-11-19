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
#include "viennamini/device_collection.hpp"


int main()
{
  viennamini::simulator  mysim;

  mysim.material_library().read("../../examples/materials.xml");

  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().scale(1.0E-9);
  mysim.device().write("output");

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
  mysim.device().ND_max(left) = 1.0E24;
  mysim.device().NA_max(left) = 1.0E8;

  mysim.device().make_semiconductor(intrinsic);
  mysim.device().name(intrinsic)     = "left";
  mysim.device().material(intrinsic) = "Si";
  mysim.device().ND_max(intrinsic) = 1.0E21;
  mysim.device().NA_max(intrinsic) = 1.0E11;

  mysim.device().make_semiconductor(right);
  mysim.device().name(right)     = "right";
  mysim.device().material(right) = "Si";
  mysim.device().ND_max(right) = 1.0E24;
  mysim.device().NA_max(right) = 1.0E8;

  mysim.device().make_contact(right_contact);
  mysim.device().name(right_contact)     = "right_contact";
  mysim.device().material(right_contact) = "Cu";
  mysim.device().contact_potential(right_contact) = 0.5;
  mysim.device().workfunction(right_contact) = 0.0;

  mysim.config().temperature()                        = 300;
  mysim.config().damping()                            = 1.0;
  mysim.config().linear_breaktol()                    = 1.0E-13;
  mysim.config().linear_iterations()                  = 700;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;
  mysim.config().initial_guess_smoothing_iterations() = 4;
  mysim.config().problem()                            = viennamini::id::poisson_drift_diffusion_np();

  mysim.run();


//  viennamini::storage   mystorage;
//  viennamini::device    mydevice(mystorage);

//  // import a VTK file
//  viennamini::io::read_vtk(mydevice, "../external/ViennaDeviceCollection/nin2d/nin2d.mesh", viennagrid::config::triangular_2d());

//  // scale the mesh to the nanometer regime
//  viennamini::scale(mydevice, 1.0E-9);

//  // identify segments
//  const int left_contact     = 1;
//  const int left             = 2;
//  const int intrinsic        = 3;
//  const int right            = 4;
//  const int right_contact    = 5;

//  mydevice.make_contact(left_contact);
//  mydevice.name(left_contact)     = "left_contact";
//  mydevice.material(left_contact) = "Cu";
//  mydevice.contact_potential(left_contact) = 0.0;
//  mydevice.workfunction(left_contact) = 0.0;

//  mydevice.make_semiconductor(left);
//  mydevice.name(left)     = "left";
//  mydevice.material(left) = "Si";
//  mydevice.ND_max(left) = 1.0E24;
//  mydevice.NA_max(left) = 1.0E8;

//  mydevice.make_semiconductor(intrinsic);
//  mydevice.name(intrinsic)     = "left";
//  mydevice.material(intrinsic) = "Si";
//  mydevice.ND_max(intrinsic) = 1.0E21;
//  mydevice.NA_max(intrinsic) = 1.0E11;

//  mydevice.make_semiconductor(right);
//  mydevice.name(right)     = "right";
//  mydevice.material(right) = "Si";
//  mydevice.ND_max(right) = 1.0E24;
//  mydevice.NA_max(right) = 1.0E8;

//  mydevice.make_contact(right_contact);
//  mydevice.name(right_contact)     = "right_contact";
//  mydevice.material(right_contact) = "Cu";
//  mydevice.contact_potential(right_contact) = 0.5;
//  mydevice.workfunction(right_contact) = 0.0;

//  // setup a the simuulation configuration 
//  viennamini::config myconfig;
//  myconfig.temperature()                        = 300;
//  myconfig.damping()                            = 1.0;
//  myconfig.linear_breaktol()                    = 1.0E-13;
//  myconfig.linear_iterations()                  = 700;
//  myconfig.nonlinear_iterations()               = 100;
//  myconfig.nonlinear_breaktol()                 = 1.0E-3;
//  myconfig.initial_guess_smoothing_iterations() = 4;

//  // prepare material library
//  viennamini::material_library mymatlib;
//  mymatlib.load("/home/weinbub/git/ViennaMini/external/ViennaMaterials/database/materials.xml");

//  // create a simulator object
//  viennamini::simulator   mysim(mymatlib /*, observer here? */);

//  // run the simulation
//  mysim(mydevice, myconfig);
  
  
 /*
 viennashe::simulator<device_type> mysim(device, config_optional);
 mysim.set_initial_guess(..);
 mysim.run()
 
 
 viennamini::simulator  mysim; // <- storage, device, config
 
 mysim.load_material_library();
 mysim.config()
 mysim.device().generate(new viennamini::capacitor2d);
 
 mysim.run()
 */
  

  std::cout << "********************************************" << std::endl;
  std::cout << "* NIN2D simulation finished successfully! *" << std::endl;
  std::cout << "********************************************" << std::endl;
  return EXIT_SUCCESS;
}


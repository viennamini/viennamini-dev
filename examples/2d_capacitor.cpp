/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
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
//
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"


int main()
{
  // create the simulator object
  //
  viennamini::simulator  mysim;

  // read mesh and material input files
  //
  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_database("../../auxiliary/materials.xml");
  mysim.device().read_unit_database("../../auxiliary/units.xml");

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
  mysim.device().scale(1.0E-9);

  // setup auxiliary segment indices, aiding in identifying the individual
  // device segments in the subsequent device setup step
  //
  const int left_contact     = 1;
  const int left_oxide       = 2;
  const int middle_oxide     = 3;
  const int right_oxide      = 4;
  const int right_contact    = 5;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::contact,    left_contact,  "left_contact",  "Cu");
  mysim.device().make(viennamini::role::oxide,      left_oxide,    "left_oxide",    "SiO2");
  mysim.device().make(viennamini::role::oxide,      middle_oxide,  "middle_oxide",  "HfO2");
  mysim.device().make(viennamini::role::oxide,      right_oxide,   "right_oxide",   "SiO2");
  mysim.device().make(viennamini::role::contact,    right_contact, "right_contact", "Cu");

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().pdeset_id()         = viennamini::pdeset::laplace;
  mysim.config().model().discretization_id() = viennamini::discret::fvm;

  // manually set the contact potentials
  //
  mysim.device().set_contact_quantity(viennamini::id::potential(), left_contact,  0.0, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), right_contact, 1.0, "V");

  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}


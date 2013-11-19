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


//const int gate_contact    = 1;
//const int source_contact  = 2;
//const int oxide           = 3;
//const int drain_contact   = 4;
//const int source          = 5;
//const int drain           = 6;
//const int body            = 7;
//const int body_contact    = 8;

///** @brief Structure the device by assigning 'roles', such as 'Oxide' to a segment.
//    Also, assign a doping to the semiconductor regions */
//template<typename MeshT, typename SegmentationT, typename StorageT>
//void prepare(viennamini::device<MeshT, SegmentationT, StorageT>& device)
//{
//  // Segment 0: Gate Contact
//  device.assign_name          (gate_contact, "gate_contact");
//  device.assign_material      (gate_contact, "Cu");
//  device.assign_contact       (gate_contact);

//  // Segment 1: Source Contact
//  device.assign_name          (source_contact, "source_contact");
//  device.assign_material      (source_contact, "Cu");
//  device.assign_contact       (source_contact);

//  // Segment 2: Oxide
//  device.assign_name          (oxide, "oxide");
//  device.assign_material      (oxide, "HfO2");
//  device.assign_oxide         (oxide);

//  // Segment 3: Drain Contact
//  device.assign_name          (drain_contact, "drain_contact");
//  device.assign_material      (drain_contact, "Cu");
//  device.assign_contact       (drain_contact);

//  // Segment 4: Source
//  device.assign_name          (source, "source");
//  device.assign_material      (source, "Si");
//  device.assign_semiconductor (source, 1.E24, 1.E8);

//  // Segment 5: Drain
//  device.assign_name          (drain, "drain");
//  device.assign_material      (drain, "Si");
//  device.assign_semiconductor (drain, 1.E24, 1.E8);

//  // Segment 6: Body
//  device.assign_name          (body, "body");
//  device.assign_material      (body, "Si");
//  device.assign_semiconductor (body, 1.E12, 1.E20);

//  // Segment 7: Body Contact
//  device.assign_name          (body_contact, "body_contact");
//  device.assign_material      (body_contact, "Cu");
//  device.assign_contact       (body_contact);
//}

///** @brief Assign actual values to the dirichlet contacts */
//void prepare_boundary_conditions(viennamini::config& config)
//{
//  // Segment 0: Gate Contact
//  config.assign_contact(gate_contact, 0.2, 0.4);  // segment id, contact potential, workfunction

//  // Segment 7: Body Contact
//  config.assign_contact(body_contact, 0.0, 0.0);

//  // Segment 1: Source Contact
//  config.assign_contact(source_contact, 0.0, 0.0);

//  // Segment 3: Drain Contact
//  config.assign_contact(drain_contact, 0.2, 0.0);
//}

int main()
{
  viennamini::simulator  mysim;

  mysim.material_library().read("../../examples/materials.xml");

  mysim.device().read(viennamini::device_collection_path()+"/mosfet2d/mosfet2d.mesh", viennamini::triangular_2d());
  mysim.device().scale(1.0E-9);
  mysim.device().write("device");

  const int gate_contact    = 1;
  const int source_contact  = 2;
  const int oxide           = 3;
  const int drain_contact   = 4;
  const int source          = 5;
  const int drain           = 6;
  const int body            = 7;
  const int body_contact    = 8;

  mysim.device().make_contact       (gate_contact);
  mysim.device().name               (gate_contact) = "gate_contact";
  mysim.device().material           (gate_contact) = "Cu";
  mysim.device().contact_potential  (gate_contact) = 0.2;
  mysim.device().workfunction       (gate_contact) = 0.4;

  mysim.device().make_contact       (source_contact);
  mysim.device().name               (source_contact) = "source_contact";
  mysim.device().material           (source_contact) = "Cu";
  mysim.device().contact_potential  (source_contact) = 0.0;
  mysim.device().workfunction       (source_contact) = 0.0;

  mysim.device().make_oxide         (oxide);
  mysim.device().name               (oxide) = "oxide";
  mysim.device().material           (oxide) = "HfO2";

  mysim.device().make_contact       (drain_contact);
  mysim.device().name               (drain_contact) = "drain_contact";
  mysim.device().material           (drain_contact) = "Cu";
  mysim.device().contact_potential  (drain_contact) = 0.2;
  mysim.device().workfunction       (drain_contact) = 0.0;

  mysim.device().make_semiconductor (source);
  mysim.device().name               (source) = "source";
  mysim.device().material           (source) = "Si";
  mysim.device().ND_max             (source) = 1.0E24;
  mysim.device().NA_max             (source) = 1.0E8;
  
  mysim.device().make_semiconductor (drain);
  mysim.device().name               (drain) = "drain";
  mysim.device().material           (drain) = "Si";
  mysim.device().ND_max             (drain) = 1.0E24;
  mysim.device().NA_max             (drain) = 1.0E8;

  mysim.device().make_semiconductor (body);
  mysim.device().name               (body) = "body";
  mysim.device().material           (body) = "Si";
  mysim.device().ND_max             (body) = 1.0E12;
  mysim.device().NA_max             (body) = 1.0E20;

  mysim.device().make_contact       (body_contact);
  mysim.device().name               (body_contact) = "body_contact";
  mysim.device().material           (body_contact) = "Cu";
  mysim.device().contact_potential  (body_contact) = 0.0;
  mysim.device().workfunction       (body_contact) = 0.0;

  mysim.config().temperature()                        = 300;
  mysim.config().damping()                            = 0.8;
  mysim.config().linear_breaktol()                    = 1.0E-13;
  mysim.config().linear_iterations()                  = 700;
  mysim.config().nonlinear_iterations()               = 100;
  mysim.config().nonlinear_breaktol()                 = 1.0E-3;
  mysim.config().initial_guess_smoothing_iterations() = 4;
  mysim.config().problem()                            = viennamini::id::poisson_drift_diffusion_np();

  mysim.run();

  std::cout << "********************************************" << std::endl;
  std::cout << "* MOSFET simulation finished successfully! *" << std::endl;
  std::cout << "********************************************" << std::endl;

//  //
//  // Create a domain from file
//  //
//  viennamini::MeshTriangular2DType           mesh;
//  viennamini::SegmentationTriangular2DType   segments(mesh);
//  viennamini::StorageType                    storage;

//  try
//  {
//    viennagrid::io::netgen_reader my_reader;
//    my_reader(mesh, segments, "../external/ViennaDeviceCollection/mosfet2d/mosfet2d.mesh");
//  }
//  catch (...)
//  {
//    std::cerr << "File-Reader failed. Aborting program..." << std::endl;
//    return EXIT_FAILURE;
//  }

//  //
//  // scale to nanometer
//  //
//  viennagrid::scale(mesh, 1e-9);

//  //
//  // Prepare material library
//  //
//  viennamini::MatLibPugixmlType matlib;
//  matlib.load("../external/ViennaMaterials/database/materials.xml");

//  //
//  // Create a device and a config object
//  //
//  viennamini::DeviceTriangular2DType device(mesh, segments, storage);
//  viennamini::config config;

//  //
//  // Prepare device, i.e., assign doping and segment roles,
//  // e.g., oxide, semiconductor, contact
//  //
//  prepare(device);

//  //
//  // Assign contact values
//  //
//  prepare_boundary_conditions(config);

//  //
//  // Set simulation parameters
//  //
//  config.temperature()                        = 300;
//  config.damping()                            = 1.0;
//  config.linear_breaktol()                    = 1.0E-13;
//  config.linear_iterations()                  = 1000;
//  config.nonlinear_iterations()               = 100;
//  config.nonlinear_breaktol()                 = 1.0E-3;
//  config.initial_guess_smoothing_iterations() = 0;

//  //
//  // Create a simulator object
//  //
//  viennamini::SimulatorTriangular2DType sim(device, matlib, config);

//  //
//  // Run the simulation
//  //
//  sim();

//  // Write results to vtk files
//  sim.write_result("mosfet");

//  std::cout << "********************************************" << std::endl;
//  std::cout << "* MOSFET simulation finished successfully! *" << std::endl;
//  std::cout << "********************************************" << std::endl;
  return EXIT_SUCCESS;
}


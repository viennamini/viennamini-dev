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

//#define VIENNAFVM_DEBUG

// Define NDEBUG to get any reasonable performance with ublas:
#define NDEBUG

#define VIENNAMINI_DEBUG

// include necessary system headers
#include <iostream>

// ViennaMini main include:
#include "viennamini/simulator.hpp"

// Vienna Includes
#include "viennamaterials/library.hpp"
#include "viennamaterials/kernels/pugixml.hpp"

const int left_contact    = 0;
const int left_n          = 1;
const int intrinsic       = 2;
const int right_n         = 3;
const int right_contact   = 4;


/** @brief Structure the device by assigning 'roles', such as 'Oxide' to a segment.
    Also, assign a doping to the semiconductor regions */
template<typename MeshT, typename SegmentationT, typename StorageT>
void prepare(viennamini::device<MeshT, SegmentationT, StorageT>& device)
{
  // Segment 0:
  device.assign_name          (left_contact, "left_contact");
  device.assign_material      (left_contact, "Cu");
  device.assign_contact       (left_contact);

  // Segment 1:
  device.assign_name          (left_n, "left_n");
  device.assign_material      (left_n, "Si");
  device.assign_semiconductor (left_n, 1.E24, 1.E8);

  // Segment 2:
  device.assign_name          (intrinsic, "intrinsic");
  device.assign_material      (intrinsic, "Si");
  device.assign_semiconductor (intrinsic, 1.E21, 1.E11);

  // Segment 3:
  device.assign_name          (right_n, "right_n");
  device.assign_material      (right_n, "Si");
  device.assign_semiconductor (right_n, 1.E24, 1.E8);

  // Segment 4:
  device.assign_name          (right_contact, "right_contact");
  device.assign_material      (right_contact, "Cu");
  device.assign_contact       (right_contact);
}

/** @brief Assign actual values to the dirichlet contacts */
void prepare_boundary_conditions(viennamini::config& config)
{
  // Segment 0:
  config.assign_contact(left_contact, 0.0, 0.0);  // segment id, contact potential, workfunction

  // Segment 4:
  config.assign_contact(right_contact, 0.5, 0.0);
}

int main()
{
  typedef double                                                       NumericType;
  typedef viennagrid::mesh< viennagrid::config::triangular_2d >        MeshType;
  typedef viennagrid::result_of::segmentation<MeshType>::type          SegmentationType;
  typedef SegmentationType::segment_handle_type                        SegmentType;
  typedef viennadata::storage<>                                        StorageType;

  //
  // Create a domain from file
  //
  MeshType           mesh;
  SegmentationType   segments(mesh);
  StorageType        storage;

  try
  {
    viennagrid::io::netgen_reader my_reader;
    my_reader(mesh, segments,  "../examples/data/nin2d.mesh");
  }
  catch (...)
  {
    std::cerr << "File-Reader failed. Aborting program..." << std::endl;
    return EXIT_FAILURE;
  }

  //
  // scale to nanometer
  //
  viennagrid::scale(mesh, 1e-9);

  //
  // Prepare material library
  //
  typedef vmat::Library<vmat::tag::pugixml>::type  MaterialLibraryType;
  MaterialLibraryType matlib;
  matlib.load("../external/ViennaMaterials/database/materials.xml");

  //
  // Create a device and a config object
  //
  typedef viennamini::device<MeshType, SegmentationType, StorageType>   DeviceType;
  DeviceType device(mesh, segments, storage);
  viennamini::config config;

  //
  // Prepare device, i.e., assign doping and segment roles,
  // e.g., oxide, semiconductor, contact
  //
  prepare(device);

  //
  // Assign contact values
  //
  prepare_boundary_conditions(config);

  //
  // Set simulation parameters
  //
  config.temperature()                        = 300;
  config.damping()                            = 1.0;
  config.linear_breaktol()                    = 1.0E-13;
  config.linear_iterations()                  = 700;
  config.nonlinear_iterations()               = 100;
  config.nonlinear_breaktol()                 = 1.0E-3;
  config.initial_guess_smoothing_iterations() = 4;

  //
  // Create a simulator object
  //
  typedef viennamini::simulator<DeviceType, MaterialLibraryType>     SimulatorType;
  SimulatorType simulator(device, matlib, config);

  //
  // Run the simulation
  //
  simulator();

  // Write results to vtk files
  simulator.write_result("nin2d");


  std::cout << "********************************************" << std::endl;
  std::cout << "* NIN2D simulation finished successfully! *" << std::endl;
  std::cout << "********************************************" << std::endl;
  return EXIT_SUCCESS;
}


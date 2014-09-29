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


// ViennaMesh includes
//
#include "viennamesh/viennamesh.hpp"

template<typename DeviceT>
void generate_1d_device(DeviceT& device)
{
  typedef viennagrid::brep_1d_mesh GeometryMeshType;
  typedef viennagrid::result_of::segmentation<GeometryMeshType>::type GeometrySegmentationType;
  typedef viennagrid::result_of::segment_handle<GeometrySegmentationType>::type GeometrySegmentHandleType;
  typedef viennagrid::segmented_mesh<GeometryMeshType, GeometrySegmentationType> SegmentedGeometryMeshType;

  // Typedefing vertex handle and point type for geometry creation
  typedef viennagrid::result_of::point<GeometryMeshType>::type PointType;
  typedef viennagrid::result_of::vertex_handle<GeometryMeshType>::type GeometryVertexHandle;

  // creating the geometry mesh
  viennamesh::result_of::parameter_handle< SegmentedGeometryMeshType >::type geometry_handle = viennamesh::make_parameter<SegmentedGeometryMeshType>();
  GeometryMeshType & geometry = geometry_handle().mesh;
  GeometrySegmentationType & segmentation = geometry_handle().segmentation;

  GeometryVertexHandle c11 = viennagrid::make_vertex( geometry, PointType(-0.5) );
  GeometryVertexHandle c1  = viennagrid::make_vertex( geometry, PointType(0.0) );
  GeometryVertexHandle i1  = viennagrid::make_vertex( geometry, PointType(1.0) );
  GeometryVertexHandle i2  = viennagrid::make_vertex( geometry, PointType(2.0) );
  GeometryVertexHandle c2  = viennagrid::make_vertex( geometry, PointType(3.0) );
  GeometryVertexHandle c21 = viennagrid::make_vertex( geometry, PointType(3.5) );

  GeometrySegmentHandleType segment1 = segmentation.make_segment();
  viennagrid::add( segment1, c11 );
  viennagrid::add( segment1, c1 );

  GeometrySegmentHandleType segment2 = segmentation.make_segment();
  viennagrid::add( segment2, c1 );
  viennagrid::add( segment2, i1 );

  GeometrySegmentHandleType segment3 = segmentation.make_segment();
  viennagrid::add( segment3, i1 );
  viennagrid::add( segment3, i2 );

  GeometrySegmentHandleType segment4 = segmentation.make_segment();
  viennagrid::add( segment4, i2 );
  viennagrid::add( segment4, c2 );

  GeometrySegmentHandleType segment5 = segmentation.make_segment();
  viennagrid::add( segment5, c2 );
  viennagrid::add( segment5, c21 );

  viennamesh::algorithm_handle mesher( new viennamesh::make_line_mesh() );

  mesher->set_input( "mesh", geometry_handle );
  mesher->set_input( "cell_size", 0.005 );
  mesher->set_input( "make_segmented_mesh", true );
  mesher->set_input( "absolute_min_geometry_point_distance", 1e-10 );
  mesher->set_input( "relative_min_geometry_point_distance", 1e-10 );

  device.make_line1d();

  mesher->set_output( "mesh", device.get_segmesh_line_1d() );
  mesher->run();
}

int main()
{
  // create the simulator object
  //
  viennamini::simulator  mysim;

  // read mesh and material input files
  //
  mysim.device().read_material_database("../../auxiliary/materials.xml");
  mysim.device().read_unit_database("../../auxiliary/units.xml");

  generate_1d_device(mysim.device());

  // perform an optional scaling step
  // e.g., transfer device dimensions to nm regime
  //
//  mysim.device().scale(1.0E-6);

//  mysim.device().set_quantity(viennamini::id::temperature(), 223.15, "K"); // -50 degrees celsius
  mysim.device().set_quantity(viennamini::id::temperature(), 273.15, "K"); //   0 degrees celsius
//  mysim.device().set_quantity(viennamini::id::temperature(), 323.15, "K"); //  50 degrees celsius


//  mysim.device().set_quantity(viennamini::id::temperature(), 273.15, "K"); //  0 degrees celsius
//  mysim.device().set_quantity(viennamini::id::temperature(), 293.15, "K"); // 20 degrees celsius
//  mysim.device().set_quantity(viennamini::id::temperature(), 313.15, "K"); // 40 degrees celsius

  // setup auxiliary segment indices, aiding in identifying the individual
  // device segments in the subsequent device setup step
  //
  const int left_contact     = 0;
  const int left_oxide       = 1;
  const int middle_oxide     = 2;
  const int right_oxide      = 3;
  const int right_contact    = 4;

  // setup the device by identifying the individual segments
  //
  mysim.device().make(viennamini::role::contact,    left_contact,  "left_contact",  "Cu");
  mysim.device().make(viennamini::role::oxide,      left_oxide,    "left_oxide",    "SiO2");
  mysim.device().make(viennamini::role::oxide,      middle_oxide,  "middle_oxide",  "Greg1");
  mysim.device().make(viennamini::role::oxide,      right_oxide,   "right_oxide",   "SiO2");
  mysim.device().make(viennamini::role::contact,    right_contact, "right_contact", "Cu");

  // set the simulation type by choosing the PDE set and the discretization
  //
  mysim.config().model().set_pdeset(viennamini::pdeset::laplace);
  mysim.config().model().set_discretization(viennamini::discret::fvm);

  // manually set the contact potentials
  //
  mysim.device().set_contact_quantity(viennamini::id::potential(), left_contact,  0.0, "V");
  mysim.device().set_contact_quantity(viennamini::id::potential(), right_contact, 1.0E4, "V");

  // perform the simulation
  //
  mysim.run();

  return EXIT_SUCCESS;
}

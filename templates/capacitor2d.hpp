#ifndef VIENNAMINI_TEMPLATES_CAPACITOR2D_HPP
#define VIENNAMINI_TEMPLATES_CAPACITOR2D_HPP

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

#include "viennamini/device_template.hpp"

#include "viennamesh/algorithm/triangle.hpp"


namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_2d_mesh                                      MeshType;
  typedef viennagrid::result_of::point<MeshType>::type                  MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type          MeshVertexHandleType;
  typedef viennagrid::result_of::line_handle<MeshType>::type            MeshLineHandleType;
  typedef viennagrid::result_of::segmentation<MeshType>::type           SegmentationType;
  typedef viennagrid::result_of::segment_handle<SegmentationType>::type SegmentHandleType;
  typedef viennagrid::segmented_mesh<MeshType, SegmentationType>        SegmentedMeshType;

public:
  capacitor2d(std::ostream& stream = std::cout)
    : viennamini::device_template(stream)
  {
    geometry_properties()["P1"]   = point_type(0.0, 0.0);
    geometry_properties()["P2"]   = point_type(3.0, 0.0);
    geometry_properties()["P3"]   = point_type(3.0, 3.0);
    geometry_properties()["P4"]   = point_type(0.0, 3.0);
    geometry_properties()["PI1"]  = point_type(1.0, 0.0);
    geometry_properties()["PI2"]  = point_type(2.0, 0.0);
    geometry_properties()["PI3"]  = point_type(2.0, 3.0);
    geometry_properties()["PI4"]  = point_type(1.0, 3.0);
    geometry_properties()["PC1"]  = point_type(0.0, 1.0);
    geometry_properties()["PC11"] = point_type(-0.5, 1.0);
    geometry_properties()["PC12"] = point_type(-0.5, 0.0);
    geometry_properties()["PC2"]  = point_type(3.0, 2.0);
    geometry_properties()["PC21"] = point_type(3.5, 2.0);
    geometry_properties()["PC22"] = point_type(3.5, 3.0);

    // general mesh generation settings
    mesher_ = viennamesh::algorithm_handle( new viennamesh::triangle::algorithm() );
    mesher_->set_input( "cell_size", 0.1 );
    mesher_->set_input( "min_angle", 0.35 );     // in radiant
    mesher_->set_input( "delaunay", true  );
//    mesher_->set_input( "algorithm_type", "incremental_delaunay" );

    contact_a_ = "ContactA";
    plate_a_   = "PlateA";
    insulator_ = "Insulator";
    plate_b_   = "PlateB";
    contact_b_ = "ContactB";

    problem_id_ = viennamini::id::laplace();

    device_handle_  = viennamini::device_handle(new viennamini::device(this->stream()));
    config_handle_  = viennamini::config_handle(new viennamini::config(this->stream()));
  }

  ~capacitor2d()
  {
  }

  /* virtual */
  void generate()
  {
    device_handle_->make_triangular2d();
    this->generate_mesh();
    device_handle_->update_problem_description();
    this->assign_segments();
  }

private:
  void generate_mesh()
  {
    viennamesh::result_of::parameter_handle< SegmentedMeshType >::type geometry_handle = viennamesh::make_parameter<SegmentedMeshType>();
    MeshType          & geometry      = geometry_handle().mesh;
    SegmentationType  & segmentation  = geometry_handle().segmentation;


    viennamesh::result_of::parameter_handle< MeshType >::type   mesh = viennamesh::make_parameter<MeshType>();

    MeshVertexHandleType p1  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P1"] [0], geometry_properties()["P1"] [1]) );
    MeshVertexHandleType p2  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P2"] [0], geometry_properties()["P2"] [1]) );
    MeshVertexHandleType p3  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P3"] [0], geometry_properties()["P3"] [1]) );
    MeshVertexHandleType p4  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P4"] [0], geometry_properties()["P4"] [1]) );
    MeshVertexHandleType pi1 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI1"][0], geometry_properties()["PI1"][1]) );
    MeshVertexHandleType pi2 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI2"][0], geometry_properties()["PI2"][1]) );
    MeshVertexHandleType pi3 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI3"][0], geometry_properties()["PI3"][1]) );
    MeshVertexHandleType pi4 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI4"][0], geometry_properties()["PI4"][1]) );
    MeshVertexHandleType pc1 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC1"][0], geometry_properties()["PC1"][1]) );
    MeshVertexHandleType pc11 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC11"][0], geometry_properties()["PC11"][1]) );
    MeshVertexHandleType pc12 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC12"][0], geometry_properties()["PC12"][1]) );
    MeshVertexHandleType pc2 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );
    MeshVertexHandleType pc21 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC21"][0], geometry_properties()["PC21"][1]) );
    MeshVertexHandleType pc22 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC22"][0], geometry_properties()["PC22"][1]) );


    // Segment 1
    std::vector<MeshLineHandleType> lines;
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(geometry, p1,   pc1);
    lines[1] = viennagrid::make_line(geometry, pc1,  pc11);
    lines[2] = viennagrid::make_line(geometry, pc11, pc12 );
    lines[3] = viennagrid::make_line(geometry, pc12, p1);

    SegmentHandleType segment1 = segmentation.make_segment();
    viennagrid::add( segment1, lines[0] );
    viennagrid::add( segment1, lines[1] );
    viennagrid::add( segment1, lines[2] );
    viennagrid::add( segment1, lines[3] );

//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    // Segment 2
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(geometry, p1,  pi1);
    lines[1] = viennagrid::make_line(geometry, pi1, pi4);
    lines[2] = viennagrid::make_line(geometry, pi4, p4 );
    lines[3] = viennagrid::make_line(geometry, p4,  pc1);
    lines[4] = viennagrid::make_line(geometry, pc1, p1);

    SegmentHandleType segment2 = segmentation.make_segment();
    viennagrid::add( segment2, lines[0] );
    viennagrid::add( segment2, lines[1] );
    viennagrid::add( segment2, lines[2] );
    viennagrid::add( segment2, lines[3] );
    viennagrid::add( segment2, lines[4] );

    // Segment 3
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(geometry, pi1, pi2);
    lines[1] = viennagrid::make_line(geometry, pi2, pi3);
    lines[2] = viennagrid::make_line(geometry, pi3, pi4);
    lines[3] = viennagrid::make_line(geometry, pi4, pi1);

    SegmentHandleType segment3 = segmentation.make_segment();
    viennagrid::add( segment3, lines[0] );
    viennagrid::add( segment3, lines[1] );
    viennagrid::add( segment3, lines[2] );
    viennagrid::add( segment3, lines[3] );

    // Segment 4
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(geometry, pi2, p2);
    lines[1] = viennagrid::make_line(geometry, p2,  pc2);
    lines[2] = viennagrid::make_line(geometry, pc2, p3);
    lines[3] = viennagrid::make_line(geometry, p3,  pi3);
    lines[4] = viennagrid::make_line(geometry, pi3, pi2);

    SegmentHandleType segment4 = segmentation.make_segment();
    viennagrid::add( segment4, lines[0] );
    viennagrid::add( segment4, lines[1] );
    viennagrid::add( segment4, lines[2] );
    viennagrid::add( segment4, lines[3] );
    viennagrid::add( segment4, lines[4] );

    // Segment 5
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(geometry, pc2,  p3);
    lines[1] = viennagrid::make_line(geometry, p3,   pc22);
    lines[2] = viennagrid::make_line(geometry, pc22, pc21 );
    lines[3] = viennagrid::make_line(geometry, pc21, pc2);

    SegmentHandleType segment5 = segmentation.make_segment();
    viennagrid::add( segment5, lines[0] );
    viennagrid::add( segment5, lines[1] );
    viennagrid::add( segment5, lines[2] );
    viennagrid::add( segment5, lines[3] );


    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", geometry_handle );


    mesher_->reference_output( "default", device_handle_->get_segmesh_triangular_2d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      stream() << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }

  void assign_segments()
  {
    device_handle_->make_contact         (0);
    device_handle_->set_name             (0, contact_a_);
    device_handle_->set_material         (0, "Cu");

    device_handle_->make_oxide           (1);
    device_handle_->set_name             (1, plate_a_);
    device_handle_->set_material         (1, "SiO2");

    device_handle_->make_semiconductor   (2);
    device_handle_->set_name             (2, insulator_);
    device_handle_->set_material         (2, "Si");

    device_handle_->make_oxide           (3);
    device_handle_->set_name             (3, plate_b_);
    device_handle_->set_material         (3, "SiO2");

    device_handle_->make_contact         (4);
    device_handle_->set_name             (4, contact_b_);
    device_handle_->set_material         (4, "Cu");
  }



private:
  viennamesh::algorithm_handle mesher_;
  std::string contact_a_;
  std::string plate_a_;
  std::string insulator_;
  std::string plate_b_;
  std::string contact_b_;
};

} // viennamini

#endif


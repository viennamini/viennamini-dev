#ifndef VIENNAMINI_TEMPLATES_CAPACITOR1D_HPP
#define VIENNAMINI_TEMPLATES_CAPACITOR1D_HPP

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

#include "viennamesh/algorithm/mesher1d.hpp"

namespace viennamini {

class capacitor1d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_1d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;
  typedef viennagrid::result_of::segmentation<MeshType>::type           SegmentationType;
  typedef viennagrid::result_of::segment_handle<SegmentationType>::type SegmentHandleType;
  typedef viennagrid::segmented_mesh<MeshType, SegmentationType>        SegmentedMeshType;

public:
  capacitor1d(std::ostream& stream = std::cout)
    : viennamini::device_template(stream)
  {
    geometry_properties()["C11"]  = point_type(-0.5);
    geometry_properties()["C1"]   = point_type(0.0);
    geometry_properties()["I1"]   = point_type(1.0);
    geometry_properties()["I2"]   = point_type(2.0);
    geometry_properties()["C2"]   = point_type(3.0);
    geometry_properties()["C21"]  = point_type(3.5);

    // general mesh generation settings
    mesher_ = viennamesh::algorithm_handle( new viennamesh::mesher1d::algorithm() );
    mesher_->set_input( "cell_size", 0.05 );

    mesher_->set_input( "absolute_min_geometry_point_distance", 1e-10 );
    mesher_->set_input( "relative_min_geometry_point_distance", 1e-10 );
    mesher_->set_input( "delaunay", true  );

    contact_a_ = "ContactA";
    plate_a_   = "PlateA";
    insulator_ = "Insulator";
    plate_b_   = "PlateB";
    contact_b_ = "ContactB";

    problem_id_ = viennamini::id::laplace();

    device_handle_  = viennamini::device_handle(new viennamini::device(this->stream()));
    config_handle_  = viennamini::config_handle(new viennamini::config(this->stream()));
  }

  ~capacitor1d()
  {
  }

  /* virtual */
  void generate()
  {
    //device_->clear(); TODO
    device_handle_->make_line1d();

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

    MeshVertexHandleType c11 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["C11"] [0]) );
    MeshVertexHandleType c1  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["C1"] [0]) );
    MeshVertexHandleType i1  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["I1"] [0]) );
    MeshVertexHandleType i2  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["I2"] [0]) );
    MeshVertexHandleType c2  = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["C2"][0]) );
    MeshVertexHandleType c21 = viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["C21"][0]) );

    SegmentHandleType segment1 = segmentation.make_segment();
    viennagrid::add( segment1, c11 );
    viennagrid::add( segment1, c1 );

    SegmentHandleType segment2 = segmentation.make_segment();
    viennagrid::add( segment2, c1 );
    viennagrid::add( segment2, i1 );

    SegmentHandleType segment3 = segmentation.make_segment();
    viennagrid::add( segment3, i1 );
    viennagrid::add( segment3, i2 );

    SegmentHandleType segment4 = segmentation.make_segment();
    viennagrid::add( segment4, i2 );
    viennagrid::add( segment4, c2 );

    SegmentHandleType segment5 = segmentation.make_segment();
    viennagrid::add( segment5, c2 );
    viennagrid::add( segment5, c21 );

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", geometry_handle );

    mesher_->reference_output( "default", device_handle_->get_segmesh_line_1d() );
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


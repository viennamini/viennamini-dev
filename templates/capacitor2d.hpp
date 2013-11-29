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
#include "viennamesh/algorithm/seed_point_locator.hpp"


namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_2d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;
  typedef viennagrid::result_of::line_handle<MeshType>::type        MeshLineHandleType;

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
    viennamesh::result_of::parameter_handle< MeshType >::type   mesh = viennamesh::make_parameter<MeshType>();

    MeshVertexHandleType p1  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P1"] [0], geometry_properties()["P1"] [1]) );
    MeshVertexHandleType p2  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P2"] [0], geometry_properties()["P2"] [1]) );
    MeshVertexHandleType p3  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P3"] [0], geometry_properties()["P3"] [1]) );
    MeshVertexHandleType p4  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P4"] [0], geometry_properties()["P4"] [1]) );
    MeshVertexHandleType pi1 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI1"][0], geometry_properties()["PI1"][1]) );
    MeshVertexHandleType pi2 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI2"][0], geometry_properties()["PI2"][1]) );
    MeshVertexHandleType pi3 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI3"][0], geometry_properties()["PI3"][1]) );
    MeshVertexHandleType pi4 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI4"][0], geometry_properties()["PI4"][1]) );
    MeshVertexHandleType pc1 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC1"][0], geometry_properties()["PC1"][1]) );
    MeshVertexHandleType pc11 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC11"][0], geometry_properties()["PC11"][1]) );
    MeshVertexHandleType pc12 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC12"][0], geometry_properties()["PC12"][1]) );
    MeshVertexHandleType pc2 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );
    MeshVertexHandleType pc21 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC21"][0], geometry_properties()["PC21"][1]) );
    MeshVertexHandleType pc22 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC22"][0], geometry_properties()["PC22"][1]) );


    // Segment 1
    std::vector<MeshLineHandleType> lines;
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh(), p1,   pc1);
    lines[1] = viennagrid::make_line(mesh(), pc1,  pc11);
    lines[2] = viennagrid::make_line(mesh(), pc11, pc12 );
    lines[3] = viennagrid::make_line(mesh(), pc12, p1);

    MeshPointType seed_point_segment_1 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    // Segment 2
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh(), p1,  pi1);
    lines[1] = viennagrid::make_line(mesh(), pi1, pi4);
    lines[2] = viennagrid::make_line(mesh(), pi4, p4 );
    lines[3] = viennagrid::make_line(mesh(), p4,  pc1);
    lines[4] = viennagrid::make_line(mesh(), pc1, p1);

    MeshPointType seed_point_segment_2 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
//    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    // Segment 3
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh(), pi1, pi2);
    lines[1] = viennagrid::make_line(mesh(), pi2, pi3);
    lines[2] = viennagrid::make_line(mesh(), pi3, pi4);
    lines[3] = viennagrid::make_line(mesh(), pi4, pi1);

    MeshPointType seed_point_segment_3 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
//    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

    // Segment 4
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh(), pi2, p2);
    lines[1] = viennagrid::make_line(mesh(), p2,  pc2);
    lines[2] = viennagrid::make_line(mesh(), pc2, p3);
    lines[3] = viennagrid::make_line(mesh(), p3,  pi3);
    lines[4] = viennagrid::make_line(mesh(), pi3, pi2);

    MeshPointType seed_point_segment_4 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

    // Segment 5
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh(), pc2,  p3);
    lines[1] = viennagrid::make_line(mesh(), p3,   pc22);
    lines[2] = viennagrid::make_line(mesh(), pc22, pc21 );
    lines[3] = viennagrid::make_line(mesh(), pc21, pc2);

    MeshPointType seed_point_segment_5 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
//    std::cout << "seed pnt 5: " << seed_point_segment_5 << std::endl;


    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

    viennamesh::seed_point_2d_container seed_points;
    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) );
    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
    seed_points.push_back( std::make_pair(seed_point_segment_4, 4) );
    seed_points.push_back( std::make_pair(seed_point_segment_5, 5) );

    // creating a parameter set object
    mesher_->set_input("seed_points", seed_points);

    mesher_->reference_output( "default", device_handle_->get_segmesh_triangular_2d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      stream() << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }

  template< typename LineIterT>
  MeshPointType compute_seed_point(MeshType const& mesh, LineIterT begin, LineIterT end)
  {
    MeshType temp_mesh;
    viennagrid::copy_element_handles(mesh, begin, end, temp_mesh, 0.0 );

    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
    seed_point_locator->set_input( "default", temp_mesh);
    seed_point_locator->run();

    typedef viennamesh::result_of::point_container<MeshPointType>::type PointContainerType;
    viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );

    if(point_container().size() != 1)
    {
      // TODO
      stream() << "Error: More than one seed point computed" << std::endl;
      exit(-1);
    }

    return point_container().front();
  }

  void assign_segments()
  {
    device_handle_->make_contact         (1);
    device_handle_->set_name             (1, contact_a_);
    device_handle_->set_material         (1, "Cu");

    device_handle_->make_oxide           (2);
    device_handle_->set_name             (2, plate_a_);
    device_handle_->set_material         (2, "SiO2");

    device_handle_->make_semiconductor   (3);
    device_handle_->set_name             (3, insulator_);
    device_handle_->set_material         (3, "Si");

    device_handle_->make_oxide           (4);
    device_handle_->set_name             (4, plate_b_);
    device_handle_->set_material         (4, "SiO2");

    device_handle_->make_contact         (5);
    device_handle_->set_name             (5, contact_b_);
    device_handle_->set_material         (5, "Cu");
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


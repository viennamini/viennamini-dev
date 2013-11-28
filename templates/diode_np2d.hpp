#ifndef VIENNAMINI_TEMPLATES_DIODENP2D_HPP
#define VIENNAMINI_TEMPLATES_DIODENP2D_HPP

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

class diode_np2d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_2d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;
  typedef viennagrid::result_of::line_handle<MeshType>::type        MeshLineHandleType;

public:
  typedef SegmentIndexMapType segment_index_map_type;

  diode_np2d(std::string const& material_library_file, std::ostream& stream = std::cout)
    : viennamini::device_template(material_library_file, stream)
  {
    geometry_properties()["P1"]   = point_type(0.0, 0.0);
    geometry_properties()["P2"]   = point_type(4.0, 0.0);
    geometry_properties()["P3"]   = point_type(4.0, 2.0);
    geometry_properties()["P4"]   = point_type(0.0, 2.0);
    geometry_properties()["PI1"]  = point_type(2.0, 0.0);
    geometry_properties()["PI2"]  = point_type(2.0, 2.0);

    geometry_properties()["PC1"]  = point_type(0.0, 1.0);
    geometry_properties()["PC11"] = point_type(-0.5, 1.0);
    geometry_properties()["P11"]  = point_type(-0.5, 0.0);

    geometry_properties()["PC2"]  = point_type(4.0, 1.0);
    geometry_properties()["PC21"] = point_type(4.5, 1.0);
    geometry_properties()["P31"]  = point_type(4.5, 2.0);

    // general mesh generation settings
    mesher_ = viennamesh::algorithm_handle( new viennamesh::triangle::algorithm() );
    mesher_->set_input( "cell_size", 0.005 );
    mesher_->set_input( "min_angle", 0.35 );     // in radiant
    mesher_->set_input( "delaunay", true  );
//    mesher_->set_input( "algorithm_type", "incremental_delaunay" );

    anode            = "Anode";
    semiconductor_p  = "P";
    semiconductor_n  = "N";
    cathode          = "Cathode";

    segment_indices_[anode]            = 1;
    segment_indices_[semiconductor_p]  = 2;
    segment_indices_[semiconductor_n]  = 3;
    segment_indices_[cathode]          = 4;
  }

  ~diode_np2d()
  {
  }

  /* virtual */
  void generate()
  {
    device_.reset();
    config_.reset();

    device_  = viennamini::device_handle(new viennamini::device(this->stream()));
    config_  = viennamini::config_handle(new viennamini::config(this->stream()));

    device_->make_triangular2d();
    device_->read_material_library(material_library_file_);
    config_->problem() = viennamini::id::poisson_drift_diffusion_np();
    config_->temperature()                        = 300;
    config_->linear_breaktol()                    = 1.0E-14;
    config_->linear_iterations()                  = 1000;
    config_->nonlinear_iterations()               = 100;
    config_->nonlinear_breaktol()                 = 1.0E-2;

    this->generate_mesh();
    device_->scale(1.0E-9);
    device_->update_problem_description();
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
    MeshVertexHandleType pc1 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC1"][0], geometry_properties()["PC1"][1]) );
    MeshVertexHandleType pc11= viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC11"][0], geometry_properties()["PC11"][1]) );
    MeshVertexHandleType p11 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P11"][0], geometry_properties()["P11"][1]) );
    MeshVertexHandleType pc2 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );
    MeshVertexHandleType pc21= viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC21"][0], geometry_properties()["PC21"][1]) );
    MeshVertexHandleType p31 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P31"][0], geometry_properties()["P31"][1]) );


    // Segment 1
    std::vector<MeshLineHandleType> lines;
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh(), p1,   pc1);
    lines[1] = viennagrid::make_line(mesh(), pc1,  pc11);
    lines[2] = viennagrid::make_line(mesh(), pc11, p11 );
    lines[3] = viennagrid::make_line(mesh(), p11,  p1);

    MeshPointType seed_point_segment_1 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    // Segment 2
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh(), p1,  pi1);
    lines[1] = viennagrid::make_line(mesh(), pi1, pi2);
    lines[2] = viennagrid::make_line(mesh(), pi2, p4 );
    lines[3] = viennagrid::make_line(mesh(), p4,  pc1);
    lines[4] = viennagrid::make_line(mesh(), pc1, p1);

    MeshPointType seed_point_segment_2 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    // Segment 3
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh(), pi1, p2);
    lines[1] = viennagrid::make_line(mesh(), p2, pc2);
    lines[2] = viennagrid::make_line(mesh(), pc2, p3);
    lines[3] = viennagrid::make_line(mesh(), p3, pi2);
    lines[4] = viennagrid::make_line(mesh(), pi2, pi1);

    MeshPointType seed_point_segment_3 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

    // Segment 4
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh(), pc2, pc21);
    lines[1] = viennagrid::make_line(mesh(), pc21,p31);
    lines[2] = viennagrid::make_line(mesh(), p31, p3);
    lines[3] = viennagrid::make_line(mesh(), p3,  pc2);

    MeshPointType seed_point_segment_4 = this->compute_seed_point(mesh(), lines.begin(), lines.end());
    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

    viennamesh::seed_point_2d_container seed_points;
    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) );
    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
    seed_points.push_back( std::make_pair(seed_point_segment_4, 4) );

    // creating a parameter set object
    mesher_->set_input("seed_points", seed_points);

    mesher_->reference_output( "default", device_->get_segmesh_triangular_2d() );
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
    device_->make_contact         (segment_indices_[anode]);
    device_->set_name             (segment_indices_[anode], anode);
    device_->set_material         (segment_indices_[anode], "Cu");

    device_->make_semiconductor   (segment_indices_[semiconductor_p]);
    device_->set_name             (segment_indices_[semiconductor_p], semiconductor_p);
    device_->set_material         (segment_indices_[semiconductor_p], "Si");
    device_->set_donator_doping   (segment_indices_[semiconductor_p], 1.0E11);
    device_->set_acceptor_doping  (segment_indices_[semiconductor_p], 1.0E21);

    device_->make_semiconductor   (segment_indices_[semiconductor_n]);
    device_->set_name             (segment_indices_[semiconductor_n], semiconductor_n);
    device_->set_material         (segment_indices_[semiconductor_n], "Si");
    device_->set_donator_doping   (segment_indices_[semiconductor_n], 1.0E21);
    device_->set_acceptor_doping  (segment_indices_[semiconductor_n], 1.0E11);

    device_->make_contact         (segment_indices_[cathode]);
    device_->set_name             (segment_indices_[cathode], cathode);
    device_->set_material         (segment_indices_[cathode], "Cu");
  }

public:
  std::string anode;
  std::string semiconductor_p;
  std::string semiconductor_n;
  std::string cathode;


private:
  viennamesh::algorithm_handle mesher_;
};

} // viennamini

#endif


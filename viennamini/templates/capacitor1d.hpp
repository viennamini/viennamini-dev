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
#include "viennamesh/algorithm/seed_point_locator.hpp"

namespace viennamini {

class capacitor1d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_1d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;

public:
  capacitor1d(std::string const& material_library_file) : viennamini::device_template(material_library_file)
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
    
    
    const char* desc = "\
--------------------------------------------------------------------------- \n \
Capacitor 1D Device Description: \n \
One-dimensional capacitor with 5 segments, geometry description as follows: \n \
\n \
C11 - S1 - C1 -- S2 -- I1 -- S3 -- I2 -- S4 -- C2 - S5 - C21\
\n";
    description_ = std::string(desc);
    description_ += " Segment Overview: \n";
    description_ += "  S1: " + contact_a_ + "\n" 
                "  S2: " + plate_a_   + "\n"
                "  S3: " + insulator_ + "\n"
                "  S4: " + plate_b_   + "\n"
                "  S5: " + contact_b_ + "\n";
    description_ += "--------------------------------------------------------------------------- \n";
  }

  ~capacitor1d()
  {
  }

  /* virtual */ 
  void generate()
  {
    device_.reset();
    config_.reset();

    device_  = viennamini::device_handle(new viennamini::device());
    config_  = viennamini::config_handle(new viennamini::config());

    device_->make_line1d();
    device_->description() = description_;
    device_->read_material_library(material_library_file_);
    config_->problem() = viennamini::id::laplace();

    this->generate_mesh();
    device_->update_problem_description();
    this->assign_segments();
  }
  
private:
  void generate_mesh()
  {
    viennamesh::result_of::parameter_handle< MeshType >::type   mesh = viennamesh::make_parameter<MeshType>();

    MeshVertexHandleType c11 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["C11"] [0]) );
    MeshVertexHandleType c1  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["C1"] [0]) );
    MeshVertexHandleType i1  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["I1"] [0]) );
    MeshVertexHandleType i2  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["I2"] [0]) );
    MeshVertexHandleType c2  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["C2"][0]) );
    MeshVertexHandleType c21 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["C21"][0]) );

    MeshPointType seed_point_segment_1 = this->compute_seed_point(mesh(), c11, c1);
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    MeshPointType seed_point_segment_2 = this->compute_seed_point(mesh(), c1, i1);
//    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;
    
    MeshPointType seed_point_segment_3 = this->compute_seed_point(mesh(), i1, i2);
//    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

    MeshPointType seed_point_segment_4 = this->compute_seed_point(mesh(), i2, c2);
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

    MeshPointType seed_point_segment_5 = this->compute_seed_point(mesh(), c2, c21);
//    std::cout << "seed pnt 5: " << seed_point_segment_5 << std::endl;


    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

    viennamesh::seed_point_1d_container seed_points;
    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) ); 
    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
    seed_points.push_back( std::make_pair(seed_point_segment_4, 4) );
    seed_points.push_back( std::make_pair(seed_point_segment_5, 5) );

    // creating a parameter set object
    mesher_->set_input("seed_points", seed_points);  

    mesher_->reference_output( "default", device_->get_segmesh_line_1d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      std::cout << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }
  
  MeshPointType compute_seed_point(MeshType const& mesh, MeshVertexHandleType & v1, MeshVertexHandleType v2)
  {
    MeshType temp_mesh;
    viennagrid::make_vertex(temp_mesh, viennagrid::point(mesh, v1));
    viennagrid::make_vertex(temp_mesh, viennagrid::point(mesh, v2));

    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
    seed_point_locator->set_input( "default", temp_mesh);
    if(!seed_point_locator->run())
    {
      // TODO provide exception
      std::cout << "Error: Seed point locator failed" << std::endl;
      exit(-1);
    }
    
    typedef viennamesh::result_of::point_container<MeshPointType>::type PointContainerType;
    viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );
    
    if(point_container().size() != 1)
    {
      // TODO provide exception
      std::cout << "Error: More than one seed point computed" << std::endl;
      exit(-1);
    }
    
    return point_container().front();
  }

  void assign_segments()
  {
    device_->make_contact         (1);
    device_->set_name             (1, contact_a_);
    device_->set_material         (1, "Cu");
    device_->set_contact_potential(1, 1.0);
    
    device_->make_semiconductor   (2);
    device_->set_name             (2, plate_a_);
    device_->set_material         (2, "SiO2");

    device_->make_semiconductor   (3);
    device_->set_name             (3, insulator_);
    device_->set_material         (3, "Si");
    
    device_->make_semiconductor   (4);
    device_->set_name             (4, plate_b_);
    device_->set_material         (4, "SiO2");

    device_->make_contact         (5);
    device_->set_name             (5, contact_b_);
    device_->set_material         (5, "Cu");
    device_->set_contact_potential(5, 0.0);
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


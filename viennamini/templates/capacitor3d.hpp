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

#include "viennamesh/algorithm/tetgen.hpp"
#include "viennamesh/algorithm/seed_point_locator.hpp"


namespace viennamini {

class capacitor3d : public viennamini::device_template
{
private:
  typedef viennagrid::brep_3d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;
  typedef viennagrid::result_of::line_handle<MeshType>::type        MeshLineHandleType;

public:
  capacitor3d(std::string const& material_library_file) : viennamini::device_template(material_library_file)
  {
    geometry_properties()["P1"]   = point_type(0.0, 0.0, 0.0);
    geometry_properties()["P2"]   = point_type(3.0, 0.0, 0.0);
    geometry_properties()["P3"]   = point_type(3.0, 1.0, 0.0);
    geometry_properties()["P4"]   = point_type(0.0, 1.0, 0.0);
    geometry_properties()["P5"]   = point_type(0.0, 0.0, 2.0);
    geometry_properties()["P6"]   = point_type(3.0, 0.0, 2.0);
    geometry_properties()["P7"]   = point_type(3.0, 1.0, 2.0);
    geometry_properties()["P8"]   = point_type(0.0, 1.0, 2.0);

    geometry_properties()["PI1"]   = point_type(1.0, 0.0, 0.0);
    geometry_properties()["PI2"]   = point_type(2.0, 0.0, 0.0);
    geometry_properties()["PI3"]   = point_type(2.0, 1.0, 0.0);
    geometry_properties()["PI4"]   = point_type(1.0, 1.0, 0.0);
    geometry_properties()["PI5"]   = point_type(1.0, 0.0, 2.0);
    geometry_properties()["PI6"]   = point_type(2.0, 0.0, 2.0);
    geometry_properties()["PI7"]   = point_type(2.0, 1.0, 2.0);
    geometry_properties()["PI8"]   = point_type(1.0, 1.0, 2.0);
    
    geometry_properties()["PC1"]   = point_type(0.0, 0.3, 0.0);
    geometry_properties()["PC3"]   = point_type(0.0, 0.3, 0.3);
    geometry_properties()["PC4"]   = point_type(0.0, 0.0, 0.3);
    geometry_properties()["P11"]   = point_type(-0.3, 0.0, 0.0);
    geometry_properties()["PC11"]  = point_type(-0.3, 0.3, 0.0);
    geometry_properties()["PC31"]  = point_type(-0.3, 0.3, 0.3);
    geometry_properties()["PC41"]  = point_type(-0.3, 0.0, 0.3);

    geometry_properties()["PC2"]   = point_type(3.0, 0.7, 1.7);
    geometry_properties()["PC5"]   = point_type(3.0, 1.0, 1.7);
    geometry_properties()["PC6"]   = point_type(3.0, 0.7, 2.0);
    geometry_properties()["P71"]   = point_type(3.3, 1.0, 2.0);
    geometry_properties()["PC21"]  = point_type(3.3, 0.7, 1.7);
    geometry_properties()["PC51"]  = point_type(3.3, 1.0, 1.7);
    geometry_properties()["PC61"]  = point_type(3.3, 0.7, 2.0);
    
    // general mesh generation settings
    mesher_ = viennamesh::algorithm_handle( new viennamesh::tetgen::algorithm() );
    mesher_->set_input( "cell_size", 1.0 );      
//    mesher_->set_input( "max_radius_edge_ratio", 1.5 );  // maximum radius edge ratio
//    mesher_->set_input( "min_dihedral_angle", 0.17 );     // minimum dihedral angle in radiant, 0.17 are about 10 degrees
    mesher_->set_input( "delaunay", true  );    
//    mesher_->set_input( "algorithm_type", "incremental_delaunay" ); 
    
    contact_a_ = "ContactA";
    plate_a_   = "PlateA";
    insulator_ = "Insulator";
    plate_b_   = "PlateB";
    contact_b_ = "ContactB";
  }

  ~capacitor3d()
  {
  }

  /* virtual */ 
  void generate()
  {
    device_.reset();
    config_.reset();

    device_  = viennamini::device_handle(new viennamini::device());
    config_  = viennamini::config_handle(new viennamini::config());

    device_->make_tetrahedral3d();
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

    MeshVertexHandleType p1  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P1"]) );
//    MeshVertexHandleType p2  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P2"]) );
//    MeshVertexHandleType p3  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P3"]) );
    MeshVertexHandleType p4  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P4"]) );
    MeshVertexHandleType p5  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P5"]) );
//    MeshVertexHandleType p6  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P6"]) );
//    MeshVertexHandleType p7  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P7"]) );
    MeshVertexHandleType p8  = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P8"]) );

    MeshVertexHandleType pi1 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI1"]) );
    MeshVertexHandleType pi2 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI2"]) );
    MeshVertexHandleType pi3 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI3"]) );
    MeshVertexHandleType pi4 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI4"]) );
    MeshVertexHandleType pi5 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI5"]) );
    MeshVertexHandleType pi6 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI6"]) );
    MeshVertexHandleType pi7 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI7"]) );
    MeshVertexHandleType pi8 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI8"]) );

//    MeshVertexHandleType pc1 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC1"]) );
//    MeshVertexHandleType pc3 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC3"]) );
//    MeshVertexHandleType pc4 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC4"]) );
//    MeshVertexHandleType p11 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P11"]) );
//    MeshVertexHandleType pc11 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC11"]) );
//    MeshVertexHandleType pc31 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC31"]) );
//    MeshVertexHandleType pc41 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC41"]) );

//    MeshVertexHandleType pc2 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC2"]) );
//    MeshVertexHandleType pc5 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC5"]) );
//    MeshVertexHandleType pc6 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC6"]) );
//    MeshVertexHandleType p71 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P71"]) );
//    MeshVertexHandleType pc21 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC21"]) );
//    MeshVertexHandleType pc51 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC51"]) );
//    MeshVertexHandleType pc61 = viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC61"]) );

//    MeshPointType seed_point_segment_1 = make_brick_and_seed_point(mesh, pc41, pc4, pc3, pc31, p11, p1, pc1, pc11);
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    MeshPointType seed_point_segment_2 = make_brick_and_seed_point(mesh, p5, pi5, pi8, p8, p1, pi1, pi4, p4);
    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    MeshPointType seed_point_segment_3 = make_brick_and_seed_point(mesh, pi5, pi6, pi7, pi8, pi1, pi2, pi3, pi4);
    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

//    MeshPointType seed_point_segment_4 = make_brick_and_seed_point(mesh, pi6, p6, p7, pi7, pi2, p2, p3, pi3);
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

//    MeshPointType seed_point_segment_5 = make_brick_and_seed_point(mesh, pc6, pc61, p71, p7, pc2, pc21, pc51, pc5);
//    std::cout << "seed pnt 5: " << seed_point_segment_5 << std::endl;

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

//    viennamesh::seed_point_3d_container seed_points;
////    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) ); 
//    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
//    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
////    seed_points.push_back( std::make_pair(seed_point_segment_4, 4) );
////    seed_points.push_back( std::make_pair(seed_point_segment_5, 5) );
//    mesher_->set_input("seed_points", seed_points);  

    mesher_->reference_output( "default", device_->get_segmesh_tetrahedral_3d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      std::cout << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }

  void assign_segments()
  {
//    device_->make_contact         (1);
//    device_->set_name             (1, contact_a_);
//    device_->set_material         (1, "Cu");
//    device_->set_contact_potential(1, 1.0);
//    
//    device_->make_semiconductor   (2);
//    device_->set_name             (2, plate_a_);
//    device_->set_material         (2, "SiO2");

//    device_->make_semiconductor   (3);
//    device_->set_name             (3, insulator_);
//    device_->set_material         (3, "Si");
//    
//    device_->make_semiconductor   (4);
//    device_->set_name             (4, plate_b_);
//    device_->set_material         (4, "SiO2");

//    device_->make_contact         (5);
//    device_->set_name             (5, contact_b_);
//    device_->set_material         (5, "Cu");
//    device_->set_contact_potential(5, 0.0);
  }
  
private:
  template<typename MeshHandleT>
  MeshPointType   make_brick_and_seed_point(MeshHandleT& mesh, 
                  MeshVertexHandleType& p1, MeshVertexHandleType& p2, MeshVertexHandleType& p3, MeshVertexHandleType& p4, // top    - counter-clockwise
                  MeshVertexHandleType& p5, MeshVertexHandleType& p6, MeshVertexHandleType& p7, MeshVertexHandleType& p8) // bottom - counter-clockwise
  {
    typedef typename viennagrid::result_of::plc_handle<MeshType>::type  PLCHandleType;
    PLCHandleType      plcs[6];
    MeshLineHandleType lines[12];

    std::cout << "make brick --- " << std::endl;
    std::cout << viennagrid::point(mesh(), p1) << std::endl;
    std::cout << viennagrid::point(mesh(), p2) << std::endl;
    std::cout << viennagrid::point(mesh(), p3) << std::endl;
    std::cout << viennagrid::point(mesh(), p4) << std::endl;
    std::cout << viennagrid::point(mesh(), p5) << std::endl;
    std::cout << viennagrid::point(mesh(), p6) << std::endl;
    std::cout << viennagrid::point(mesh(), p7) << std::endl;
    std::cout << viennagrid::point(mesh(), p8) << std::endl;

    // top
    lines[0] = viennagrid::make_line(mesh(), p1, p2);
    lines[1] = viennagrid::make_line(mesh(), p2, p3);
    lines[2] = viennagrid::make_line(mesh(), p3, p4 );
    lines[3] = viennagrid::make_line(mesh(), p4, p1);
    
    // bottom
    lines[4] = viennagrid::make_line(mesh(), p5, p6);
    lines[5] = viennagrid::make_line(mesh(), p6, p7);
    lines[6] = viennagrid::make_line(mesh(), p7, p8 );
    lines[7] = viennagrid::make_line(mesh(), p8, p5);
    
    // top-bottom connections
    lines[8] = viennagrid::make_line(mesh(), p1, p5);
    lines[9] = viennagrid::make_line(mesh(), p2, p6);
    lines[10] = viennagrid::make_line(mesh(), p3, p7 );
    lines[11] = viennagrid::make_line(mesh(), p4, p8);
    
    // top plc
    plcs[0] = viennagrid::make_plc( mesh(), lines+0, lines+4 );
    
    // bottom plc
    plcs[1] = viennagrid::make_plc( mesh(), lines+4, lines+8 );

    // face 1
    {
      MeshLineHandleType cur_lines[4];
      cur_lines[0] = lines[0];
      cur_lines[1] = lines[9];
      cur_lines[2] = lines[4];
      cur_lines[3] = lines[8];
      plcs[2] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
    }

    // face 2
    {
      MeshLineHandleType cur_lines[4];
      cur_lines[0] = lines[1];
      cur_lines[1] = lines[10];
      cur_lines[2] = lines[5];
      cur_lines[3] = lines[9];
      plcs[3] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
    }

    // face 3
    {
      MeshLineHandleType cur_lines[4];
      cur_lines[0] = lines[2];
      cur_lines[1] = lines[10];
      cur_lines[2] = lines[6];
      cur_lines[3] = lines[11];
      plcs[4] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
    }

    // face 4
    {
      MeshLineHandleType cur_lines[4];
      cur_lines[0] = lines[3];
      cur_lines[1] = lines[11];
      cur_lines[2] = lines[7];
      cur_lines[3] = lines[8];
      plcs[5] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
    }
    
//    MeshType temp_mesh;
//    viennagrid::copy_element_handles( mesh(), plcs+0, plcs+6, temp_mesh, 0.0 );
//    
//    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
//    seed_point_locator->set_input( "default", temp_mesh);
//    seed_point_locator->run();
//    
//    typedef viennamesh::result_of::point_container<MeshPointType>::type PointContainerType;
//    viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );
//    
//    if(point_container().size() != 1)
//    {
//      // TODO
//      std::cout << "Error: More than one seed point computed" << std::endl;
//      exit(-1);
//    }
//    
//    return point_container().front();
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


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
  typedef viennagrid::plc_2d_mesh                                  MeshType;
  typedef viennagrid::result_of::point<MeshType>::type              MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type      MeshVertexHandleType;
  typedef viennagrid::result_of::line_handle<MeshType>::type        MeshLineHandleType;

public:
  capacitor2d(std::string const& material_library_file) : viennamini::device_template(material_library_file)
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
    mesher_->set_input( "algorithm_type", "incremental_delaunay" ); 
    
    contact_a_ = "ContactA";
    plate_a_   = "PlateA";
    insulator_ = "Insulator";
    plate_b_   = "PlateB";
    contact_b_ = "ContactB";
    
    
    const char* desc = "\
--------------------------------------------------------------------------- \n \
Capacitor 2D Device Description: \n \
Two dimensional capacitor with 5 segments, geometry description as follows: \n \
\n \
        P4 --- PI4 --- PI3 --- P3   -- PC22 \n \
        |       |       |       |  S5   |   \n \
PC11 -- PC1 S2  |  S3   |  S4   PC2 -- PC21 \n \
|   S1  |       |       |       |           \n \
PC12 -- P1 --- PI1 --- PI2 --- P2           \n \
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

  ~capacitor2d()
  {
  }

  /* virtual */ 
  void generate()
  {
    device_.reset();
    config_.reset();

    device_  = viennamini::device_handle(new viennamini::device());
    config_  = viennamini::config_handle(new viennamini::config());

    device_->make_triangular2d();
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

    MeshVertexHandleType p1  = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["P1"] [0], geometry_properties()["P1"] [1]) );
    MeshVertexHandleType p2  = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["P2"] [0], geometry_properties()["P2"] [1]) );
    MeshVertexHandleType p3  = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["P3"] [0], geometry_properties()["P3"] [1]) );
    MeshVertexHandleType p4  = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["P4"] [0], geometry_properties()["P4"] [1]) );
    MeshVertexHandleType pi1 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PI1"][0], geometry_properties()["PI1"][1]) );
    MeshVertexHandleType pi2 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PI2"][0], geometry_properties()["PI2"][1]) );
    MeshVertexHandleType pi3 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PI3"][0], geometry_properties()["PI3"][1]) );
    MeshVertexHandleType pi4 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PI4"][0], geometry_properties()["PI4"][1]) );
    MeshVertexHandleType pc1 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC1"][0], geometry_properties()["PC1"][1]) );
    MeshVertexHandleType pc11 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC11"][0], geometry_properties()["PC11"][1]) );
    MeshVertexHandleType pc12 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC12"][0], geometry_properties()["PC12"][1]) );
    MeshVertexHandleType pc2 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );
    MeshVertexHandleType pc21 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC21"][0], geometry_properties()["PC21"][1]) );
    MeshVertexHandleType pc22 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC22"][0], geometry_properties()["PC22"][1]) );


    // Segment 1
    std::vector<MeshLineHandleType> lines;
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh->get(), p1,   pc1);
    lines[1] = viennagrid::make_line(mesh->get(), pc1,  pc11);
    lines[2] = viennagrid::make_line(mesh->get(), pc11, pc12 );
    lines[3] = viennagrid::make_line(mesh->get(), pc12, p1);

    MeshPointType seed_point_segment_1 = this->compute_seed_point(mesh->get(), lines.begin(), lines.end());
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    // Segment 2
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh->get(), p1,  pi1);
    lines[1] = viennagrid::make_line(mesh->get(), pi1, pi4);
    lines[2] = viennagrid::make_line(mesh->get(), pi4, p4 );
    lines[3] = viennagrid::make_line(mesh->get(), p4,  pc1);
    lines[4] = viennagrid::make_line(mesh->get(), pc1, p1);

    MeshPointType seed_point_segment_2 = this->compute_seed_point(mesh->get(), lines.begin(), lines.end());
//    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    // Segment 3
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh->get(), pi1, pi2);
    lines[1] = viennagrid::make_line(mesh->get(), pi2, pi3);
    lines[2] = viennagrid::make_line(mesh->get(), pi3, pi4);
    lines[3] = viennagrid::make_line(mesh->get(), pi4, pi1);
    
    MeshPointType seed_point_segment_3 = this->compute_seed_point(mesh->get(), lines.begin(), lines.end());
//    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

    // Segment 4
    lines.clear();
    lines.resize(5);
    lines[0] = viennagrid::make_line(mesh->get(), pi2, p2);
    lines[1] = viennagrid::make_line(mesh->get(), p2,  pc2);
    lines[2] = viennagrid::make_line(mesh->get(), pc2, p3);
    lines[3] = viennagrid::make_line(mesh->get(), p3,  pi3);
    lines[4] = viennagrid::make_line(mesh->get(), pi3, pi2);

    MeshPointType seed_point_segment_4 = this->compute_seed_point(mesh->get(), lines.begin(), lines.end());
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

    // Segment 5
    lines.clear();
    lines.resize(4);
    lines[0] = viennagrid::make_line(mesh->get(), pc2,  p3);
    lines[1] = viennagrid::make_line(mesh->get(), p3,   pc22);
    lines[2] = viennagrid::make_line(mesh->get(), pc22, pc21 );
    lines[3] = viennagrid::make_line(mesh->get(), pc21, pc2);

    MeshPointType seed_point_segment_5 = this->compute_seed_point(mesh->get(), lines.begin(), lines.end());
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

    mesher_->reference_output( "default", device_->get_segmesh_triangular_2d() );
    mesher_->run();
  }
  
  template<typename MeshT, typename LineIterT>
  MeshPointType compute_seed_point(MeshT const & mesh, LineIterT begin, LineIterT end)
  {
    MeshT new_mesh;
    
    typedef typename viennagrid::result_of::vertex<MeshT>::type VertexType;
    typedef typename viennagrid::result_of::id<VertexType>::type VertexIDType;
    typedef typename viennagrid::result_of::line<MeshT>::type LineType;
    
    std::vector<MeshLineHandleType> new_lines;
  
    std::map<VertexIDType, MeshVertexHandleType> vertex_map;
    for (LineIterT it = begin; it != end; ++it)
    {
      LineType const & line = viennagrid::dereference_handle( mesh, *it );
    
      MeshVertexHandleType vtx_handle[2];
    
      for (int i = 0; i < 2; ++i)
      {
        typename std::map<VertexIDType, MeshVertexHandleType>::iterator vtx_handle_it = vertex_map.find( viennagrid::vertices(line)[i].id() );
        if (vtx_handle_it == vertex_map.end())
        {
          vtx_handle[i] = viennagrid::make_vertex( new_mesh, viennagrid::point(mesh, viennagrid::vertices(line)[i]) );
          vertex_map[viennagrid::vertices(line)[i].id()] = vtx_handle[i];
        }
        else
          vtx_handle[i] = vtx_handle_it->second;
      }
        
      new_lines.push_back( viennagrid::make_line( new_mesh, vtx_handle[0], vtx_handle[1] ) );
    }
  
  
    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
    viennagrid::make_plc( new_mesh, new_lines.begin(), new_lines.end() );
    
    seed_point_locator->set_input( "default", new_mesh );
    seed_point_locator->run();
    return seed_point_locator->get_output<MeshPointType>( "default" )->get();
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


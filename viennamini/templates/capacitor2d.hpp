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
  capacitor2d() 
  {
    geometry_properties()["P1"]  = point_type(0.0, 0.0);
    geometry_properties()["P2"]  = point_type(3.0, 0.0);
    geometry_properties()["P3"]  = point_type(3.0, 3.0);
    geometry_properties()["P4"]  = point_type(0.0, 3.0);
    geometry_properties()["PI1"] = point_type(1.0, 0.0); 
    geometry_properties()["PI2"] = point_type(2.0, 0.0);
    geometry_properties()["PI3"] = point_type(2.0, 3.0);
    geometry_properties()["PI4"] = point_type(1.0, 3.0);
    geometry_properties()["PC1"] = point_type(0.0, 1.0);
    geometry_properties()["PC2"] = point_type(3.0, 2.0);
    
    // general mesh generation settings
    mesher_ = viennamesh::algorithm_handle( new viennamesh::triangle::algorithm() );
    mesher_->set_input( "cell_size", 0.1 );      
    mesher_->set_input( "min_angle", 0.35 );     // in radiant
    mesher_->set_input( "delaunay", true  );    
    mesher_->set_input( "algorithm_type", "incremental_delaunay" ); 
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
    config_->problem() = viennamini::id::laplace();

    this->generate_mesh();
    this->assign_segments();
  }
  
  /* virtual */ 
  std::string description()
  {
    return std::string("");
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
    MeshVertexHandleType pc2 = viennagrid::make_vertex( mesh->get(), MeshPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );

    // Segment 1
    std::vector<MeshLineHandleType> lines(14);
    lines[0] = viennagrid::make_line(mesh->get(), p1,  pi1);
    lines[1] = viennagrid::make_line(mesh->get(), pi1, pi4);
    lines[2] = viennagrid::make_line(mesh->get(), pi4, p4 );
    lines[3] = viennagrid::make_line(mesh->get(), p4,  pc1);
    lines[4] = viennagrid::make_line(mesh->get(), pc1, p1);

    MeshPointType seed_point_segment_0 = this->compute_seed_point(mesh->get(), lines.begin(), lines.begin()+5);
    std::cout << "seed pnt 0: " << seed_point_segment_0 << std::endl;

    // Segment 2
    lines[5] = viennagrid::make_line(mesh->get(), pi1, pi2);
    lines[6] = viennagrid::make_line(mesh->get(), pi2, pi3);
    lines[7] = viennagrid::make_line(mesh->get(), pi3, pi4);
    lines[8] = viennagrid::make_line(mesh->get(), pi4, pi1);
    
    MeshPointType seed_point_segment_1 = this->compute_seed_point(mesh->get(), lines.begin()+5, lines.begin()+9);
    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    // Segment 3
    lines[9]  = viennagrid::make_line(mesh->get(), pi2, p2);
    lines[10] = viennagrid::make_line(mesh->get(), p2,  pc2);
    lines[11] = viennagrid::make_line(mesh->get(), pc2, p3);
    lines[12] = viennagrid::make_line(mesh->get(), p3,  pi3);
    lines[13] = viennagrid::make_line(mesh->get(), pi3, pi2);

    MeshPointType seed_point_segment_2 = this->compute_seed_point(mesh->get(), lines.begin()+9, lines.begin()+14);
    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

    viennamesh::seed_point_2d_container seed_points;
    seed_points.push_back( std::make_pair(MeshPointType(0.5,0.5), 0) ); // TODO!
    seed_points.push_back( std::make_pair(MeshPointType(1.5,0.5), 1) );
    seed_points.push_back( std::make_pair(MeshPointType(2.5,0.5), 2) );

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
    device_->make_contact(0);
    device_->name(0)        = "plate_A";
    device_->material(0)    = "Cu";
    device_->contact_potential(0) = 1.0;
    
    device_->make_oxide(1);
    device_->name(1)        = "insulator";
    device_->material(1)    = "Air";

    device_->make_contact(2);
    device_->name(2)        = "plate_B";
    device_->material(2)    = "Cu";
    device_->contact_potential(2) = 0.0;
  }
  


private:
  viennamesh::algorithm_handle mesher_;
};

} // viennamini

#endif


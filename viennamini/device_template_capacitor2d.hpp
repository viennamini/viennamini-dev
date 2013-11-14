#ifndef VIENNAMINI_DEVICETEMPLATE_NIN2D_HPP
#define VIENNAMINI_DEVICETEMPLATE_NIN2D_HPP

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

#include "viennamesh/algorithm/triangle_mesher.hpp"
#include "viennagrid/config/default_configs.hpp"


namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::line_2d_mesh                                                    Line2DMeshType;
  typedef viennagrid::result_of::point<Line2DMeshType>::type                          LocalPointType;
  typedef viennagrid::result_of::handle<Line2DMeshType, viennagrid::vertex_tag>::type VertexHandleType;

public:
  capacitor2d(viennamini::StorageType& storage) : 
    viennamini::device_template(storage) 
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
    settings_.set("cell_size", 0.1);             // maximum cell size is set to 1
    settings_.set("min_angle", 30.0);            // minimum angle is set to 30
   
    device().make_triangular2d();
    
    // the segment information is static
    this->assign_segments();
  }

  virtual void generate()
  {
    this->generate_mesh();
  }
  
  
private:
  void generate_mesh()
  {
    Line2DMeshType line2d;

    VertexHandleType p1  = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["P1"] [0], geometry_properties()["P1"] [1]) );
    VertexHandleType p2  = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["P2"] [0], geometry_properties()["P2"] [1]) );
    VertexHandleType p3  = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["P3"] [0], geometry_properties()["P3"] [1]) );
    VertexHandleType p4  = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["P4"] [0], geometry_properties()["P4"] [1]) );
    VertexHandleType pi1 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PI1"][0], geometry_properties()["PI1"][1]) );
    VertexHandleType pi2 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PI2"][0], geometry_properties()["PI2"][1]) );
    VertexHandleType pi3 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PI3"][0], geometry_properties()["PI3"][1]) );
    VertexHandleType pi4 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PI4"][0], geometry_properties()["PI4"][1]) );
    VertexHandleType pc1 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PC1"][0], geometry_properties()["PC1"][1]) );
    VertexHandleType pc2 = viennagrid::make_vertex( line2d, LocalPointType(geometry_properties()["PC2"][0], geometry_properties()["PC2"][1]) );

    // Segment 1
    viennagrid::make_line(line2d, p1,  pi1);
    viennagrid::make_line(line2d, pi1, pi4);
    viennagrid::make_line(line2d, pi4, p4 );
    viennagrid::make_line(line2d, p4,  pc1);
    viennagrid::make_line(line2d, pc1, p1);
    
    // Segment 2
    viennagrid::make_line(line2d, pi1, pi2);
    viennagrid::make_line(line2d, pi2, pi3);
    viennagrid::make_line(line2d, pi3, pi4);
    viennagrid::make_line(line2d, pi4, pi1);
    
    // Segment 3
    viennagrid::make_line(line2d, pi2, p2);
    viennagrid::make_line(line2d, p2,  pc2);
    viennagrid::make_line(line2d, pc2, p3);
    viennagrid::make_line(line2d, p3,  pi3);
    viennagrid::make_line(line2d, pi3, pi2);

    viennamesh::seed_point_2d_container seed_points;
    seed_points.push_back( viennamesh::seed_point_2d_container::value_type(viennagrid::config::point_type_2d(0.5, 1.0), 0) ); // TODO!
    seed_points.push_back( viennamesh::seed_point_2d_container::value_type(viennagrid::config::point_type_2d(1.5, 1.0), 1) );
    seed_points.push_back( viennamesh::seed_point_2d_container::value_type(viennagrid::config::point_type_2d(2.5, 1.0), 2) );

    // creating a parameter set object
    settings_.set("seed_points", seed_points);   // the seed points

    viennamini::SegmentationTriangular2DType & segmentation = boost::get<viennamini::SegmentationTriangular2DType>(device().generic_segmentation());
    viennamini::MeshTriangular2DType         & mesh         = boost::get<viennamini::MeshTriangular2DType>        (device().generic_mesh());

    // starting the meshing algorithm
    viennamesh::run_algo<viennamesh::triangle_tag>(
      line2d, viennamesh::NoSegmentation(),
      mesh, segmentation,
      settings_);
  }
  
  void assign_segments()
  {
    device().make_contact(0);
    device().name(0)        = "plate_A";
    device().material(0)    = "metal";
    
    device().make_oxide(1);
    device().name(1)        = "insulator";
    device().material(1)    = "air";

    device().make_contact(2);
    device().name(2)        = "plate_B";
    device().material(2)    = "metal";
  }
  


private:
  viennamesh::ConstParameterSet settings_;
};

} // viennamini

#endif


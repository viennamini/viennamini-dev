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

#include "viennamesh/algorithm/cgal_plc_mesher.hpp"
#include "viennamesh/algorithm/triangle_mesher.hpp"
#include "viennagrid/config/default_configs.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/seed_point_segmenting.hpp"

namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::line_2d_mesh                                                    Line2DMeshType;
  typedef viennagrid::result_of::point<Line2DMeshType>::type                          LocalPointType;
  typedef viennagrid::result_of::handle<Line2DMeshType, viennagrid::vertex_tag>::type VertexHandleType;

public:
  capacitor2d()
  {
    properties_["P1"]  = point_type(0.0, 0.0);
    properties_["P2"]  = point_type(3.0, 0.0);
    properties_["P3"]  = point_type(3.0, 3.0);
    properties_["P4"]  = point_type(0.0, 3.0);
    properties_["PI1"] = point_type(1.0, 0.0); 
    properties_["PI2"] = point_type(2.0, 0.0);
    properties_["PI3"] = point_type(2.0, 3.0);
    properties_["PI4"] = point_type(1.0, 3.0);
    properties_["PC1"] = point_type(0.0, 1.0);
    properties_["PC2"] = point_type(3.0, 2.0);
  }

  virtual void generate()
  {
    Line2DMeshType line2d;

    VertexHandleType p1  = viennagrid::make_vertex( line2d, LocalPointType(properties_["P1"] [0], properties_["P1"] [1]) );
    VertexHandleType p2  = viennagrid::make_vertex( line2d, LocalPointType(properties_["P2"] [0], properties_["P2"] [1]) );
    VertexHandleType p3  = viennagrid::make_vertex( line2d, LocalPointType(properties_["P3"] [0], properties_["P3"] [1]) );
    VertexHandleType p4  = viennagrid::make_vertex( line2d, LocalPointType(properties_["P4"] [0], properties_["P4"] [1]) );
    VertexHandleType pi1 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PI1"][0], properties_["PI1"][1]) );
    VertexHandleType pi2 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PI2"][0], properties_["PI2"][1]) );
    VertexHandleType pi3 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PI3"][0], properties_["PI3"][1]) );
    VertexHandleType pi4 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PI4"][0], properties_["PI4"][1]) );
    VertexHandleType pc1 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PC1"][0], properties_["PC1"][1]) );
    VertexHandleType pc2 = viennagrid::make_vertex( line2d, LocalPointType(properties_["PC2"][0], properties_["PC2"][1]) );

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
    seed_points.push_back( viennamesh::seed_point_2d(0.5, 1.0, 0) );
    seed_points.push_back( viennamesh::seed_point_2d(1.5, 1.0, 1) );
    seed_points.push_back( viennamesh::seed_point_2d(2.5, 1.0, 2) );

    // creating a parameter set object
    viennamesh::ConstParameterSet settings;
    settings.set("seed_points", seed_points);   // the seed points
    settings.set("cell_size", 1.0);             // maximum cell size is set to 1
    settings.set("min_angle", 30.0);            // minimum angle is set to 30

    // creating a triangular mesh and segmentation
    viennagrid::triangular_2d_mesh         tri_mesh;
    viennagrid::triangular_2d_segmentation tri_segmentation(tri_mesh);

    // starting the algorithm
    viennamesh::run_algo<viennamesh::triangle_tag>(
      line2d, viennamesh::NoSegmentation(),
      tri_mesh, tri_segmentation,
      settings);

    // writing the output to a VTK file
    viennagrid::io::vtk_writer<viennagrid::triangular_2d_mesh> vtk_writer;
    vtk_writer(tri_mesh, tri_segmentation, "meshed_triangles");
  }
};

} // viennamini

#endif


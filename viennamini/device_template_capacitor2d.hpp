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
#include "viennagrid/config/default_configs.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/seed_point_segmenting.hpp"

namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::plc_2d_mesh                                              PLCType;
  typedef viennagrid::result_of::point<PLCType>::type                          LocalPointType;
  typedef viennagrid::result_of::handle<PLCType, viennagrid::vertex_tag>::type VertexHandleType;
  typedef viennagrid::result_of::handle<PLCType, viennagrid::line_tag>::type   LineHandleType;
  typedef viennagrid::result_of::handle<PLCType, viennagrid::plc_tag>::type    PLCHandleType;

public:
  capacitor2d()
  {
    properties_["P1"] = point_type(0.0, 0.0);
    properties_["P2"] = point_type(3.0, 0.0);
    properties_["P3"] = point_type(3.0, 3.0);
    properties_["P4"] = point_type(0.0, 3.0);

    properties_["PI1"] = point_type(1.0, 0.0); 
    properties_["PI2"] = point_type(2.0, 0.0);
    properties_["PI3"] = point_type(2.0, 3.0);
    properties_["PI4"] = point_type(1.0, 3.0);

    properties_["PC1"] = point_type(0.0, 1.0);
    properties_["PC2"] = point_type(3.0, 2.0);
  }

  virtual void generate()
  {
    PLCType plc;
    std::vector<LocalPointType>   hole_points;
    std::vector<VertexHandleType> points;
    std::vector<LineHandleType> lines;

    { // Segment 1
      std::vector<VertexHandleType> v;
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["P1"][0],  properties_["P1"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI1"][0], properties_["PI1"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI4"][0], properties_["PI4"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["P4"][0],  properties_["P4"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PC1"][0], properties_["PC1"][1]) ) );

//      std::vector<LineHandleType> lines;
      lines.push_back( viennagrid::make_line(plc, v[0], v[1]) );
      lines.push_back( viennagrid::make_line(plc, v[1], v[2]) );
      lines.push_back( viennagrid::make_line(plc, v[2], v[3]) );
      lines.push_back( viennagrid::make_line(plc, v[3], v[4]) );
      lines.push_back( viennagrid::make_line(plc, v[4], v[0]) );

//      viennagrid::make_plc(plc,
//        lines.begin(), lines.end(),
//        points.begin(), points.end(),
//        hole_points.begin(), hole_points.end()
//      );
    }
    { // Segment 2
      std::vector<VertexHandleType> v;
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI1"][0],  properties_["PI1"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI2"][0],  properties_["PI2"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI3"][0],  properties_["PI3"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI4"][0],  properties_["PI4"][1]) ) );

//      std::vector<LineHandleType> lines;
      lines.push_back( viennagrid::make_line(plc, v[0], v[1]) );
      lines.push_back( viennagrid::make_line(plc, v[1], v[2]) );
      lines.push_back( viennagrid::make_line(plc, v[2], v[3]) );
      lines.push_back( viennagrid::make_line(plc, v[3], v[0]) );

//      viennagrid::make_plc(plc,
//        lines.begin(), lines.end(),
//        points.begin(), points.end(),
//        hole_points.begin(), hole_points.end()
//      );
    }
    { // Segment 3
      std::vector<VertexHandleType> v;
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI2"][0],  properties_["PI2"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["P2"][0], properties_["P2"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PC2"][0], properties_["PC2"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["P3"][0],  properties_["P3"][1]) ) );
      v.push_back( viennagrid::make_vertex( plc, LocalPointType(properties_["PI3"][0], properties_["PI3"][1]) ) );

//      std::vector<LineHandleType> lines;
      lines.push_back( viennagrid::make_line(plc, v[0], v[1]) );
      lines.push_back( viennagrid::make_line(plc, v[1], v[2]) );
      lines.push_back( viennagrid::make_line(plc, v[2], v[3]) );
      lines.push_back( viennagrid::make_line(plc, v[3], v[4]) );
      lines.push_back( viennagrid::make_line(plc, v[4], v[0]) );

//      viennagrid::make_plc(plc,
//        lines.begin(), lines.end(),
//        points.begin(), points.end(),
//        hole_points.begin(), hole_points.end()
//      );
    }

    viennagrid::make_plc(plc,
      lines.begin(), lines.end(),
      points.begin(), points.end(),
      hole_points.begin(), hole_points.end()
    );

  viennamesh::ConstParameterSet settings;
  settings.set("shortes_edge_circumradius_ratio",0.3);

  viennagrid::triangular_2d_mesh              mesh;
  viennagrid::triangular_2d_segmentation segmentation(mesh);

  std::vector< std::pair< int, LocalPointType > > seed_points;
  seed_points.push_back( std::make_pair(0, LocalPointType(0.5, 0.5)) );
  seed_points.push_back( std::make_pair(1, LocalPointType(1.5, 0.5)) );
  seed_points.push_back( std::make_pair(2, LocalPointType(2.5, 0.5)) );
//  viennagrid::mark_face_segments( mesh, segmentation, seed_points.begin(), seed_points.end() );

//  viennamesh::run_algo< viennamesh::cgal_plc_2d_mesher_tag >( plc, mesh, segmentation, settings );

//  viennagrid::io::vtk_writer<viennagrid::triangular_2d_mesh> vtk_writer;
//  vtk_writer(mesh, "test");

  }
};

} // viennamini

#endif


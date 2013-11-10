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

namespace viennamini {

class capacitor2d : public viennamini::device_template
{
private:
  typedef viennagrid::plc_2d_mesh                                               MeshType;
  typedef viennagrid::result_of::point<MeshType>::type                          LocalPointType;
  typedef viennagrid::result_of::handle<MeshType, viennagrid::vertex_tag>::type VertexHandleType;

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
    MeshType mesh;
//    std::vector<line_handle_type> lines;

    { // Segment 1
      std::vector<VertexHandleType> v;
      v.push_back( viennagrid::make_vertex( mesh, LocalPointType(properties_["P1"][0],  properties_["P1"][1]) ) );
      v.push_back( viennagrid::make_vertex( mesh, LocalPointType(properties_["PI1"][0], properties_["PI1"][1]) ) );
      v.push_back( viennagrid::make_vertex( mesh, LocalPointType(properties_["PI4"][0], properties_["PI4"][1]) ) );
      v.push_back( viennagrid::make_vertex( mesh, LocalPointType(properties_["P4"][0],  properties_["P4"][1]) ) );
      v.push_back( viennagrid::make_vertex( mesh, LocalPointType(properties_["PC1"][0], properties_["PC1"][1]) ) );
    }
  }
};

} // viennamini

#endif


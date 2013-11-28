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



// This method helps us to create or fetch a line based on vertex handles
// if a line is already present, we just use that line, otherwise we create it
// We are using a std::map< std::pair<VertexID, VertexID>, LineHandle > for keeping track of all lines in the mesh
template<typename MeshT, typename VertexHandleT, typename VertexLineMapT>
typename viennagrid::result_of::line_handle<MeshT>::type get_make_line(MeshT & mesh, VertexLineMapT & vertex_line_map, VertexHandleT const & vtxh0, VertexHandleT const & vtxh1)
{
  // querying the ID of the vertices
  typedef typename viennagrid::result_of::vertex_id<MeshT>::type VertexIDType;
  VertexIDType id0 = viennagrid::dereference_handle(mesh, vtxh0).id();
  VertexIDType id1 = viennagrid::dereference_handle(mesh, vtxh1).id();

  // creating the key for the line, note that we order the IDs within the pair for uniqueness
  std::pair<VertexIDType, VertexIDType> key = std::make_pair( std::min(id0, id1), std::max(id0, id1) );

  // searching for the line in our map
  typename VertexLineMapT::iterator it = vertex_line_map.find(key);
  if (it != vertex_line_map.end())
  {
    // found -> return
    return it->second;
  }
  else
  {
    // not found -> create the line
    typename viennagrid::result_of::line_handle<MeshT>::type tmp = viennagrid::make_line( mesh, vtxh0, vtxh1 );
    vertex_line_map[ key ] = tmp;
    return tmp;
  }
}


// This method helps us creating a quad PLC based on 4 vertex handles
// we use get_make_line (see above) and a vertex IDs to line map for keeping track of all lines
template<typename MeshT, typename VertexLineMapT, typename VertexHandleT>
typename viennagrid::result_of::plc_handle<MeshT>::type make_quad_plc(MeshT & mesh, VertexLineMapT & vertex_line_map, VertexHandleT v0, VertexHandleT v1, VertexHandleT v2, VertexHandleT v3)
{
  typedef typename viennagrid::result_of::line_handle<MeshT>::type LineHandleType;
  viennagrid::static_array<LineHandleType, 4> lines;
  lines[0] = get_make_line(mesh, vertex_line_map, v0, v1);
  lines[1] = get_make_line(mesh, vertex_line_map, v1, v2);
  lines[2] = get_make_line(mesh, vertex_line_map, v2, v3);
  lines[3] = get_make_line(mesh, vertex_line_map, v3, v0);

  return viennagrid::make_plc(mesh, lines.begin(), lines.end());
}


// this method helps us finding the seed point of a geometry based on a range of boundary PLCs
template<typename MeshT, typename PLCHandleIteratorT>
typename viennagrid::result_of::point<MeshT>::type get_seed_point( MeshT const & mesh, PLCHandleIteratorT const & begin, PLCHandleIteratorT const & end)
{
  // creating a temporary mesh parameter for the seed point locator
  typename viennamesh::result_of::parameter_handle< MeshT >::type tmp_mesh = viennamesh::make_parameter<MeshT>();

  // copy all PLCs to our temporary mesh
  viennagrid::copy_element_handles(mesh, begin, end, tmp_mesh(), 0.0);

  // Create the algorithm, set the temporary mesh as an input and execute it
  viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
  seed_point_locator->set_input( "default", tmp_mesh );
  seed_point_locator->run();

  // Querying the algorithm output (a seed point) and return it
  typedef typename viennagrid::result_of::point<MeshT>::type PointType;
  typedef typename viennamesh::result_of::point_container<PointType>::type PointContainerType;
  typename viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );
  if (point_container && !point_container().empty())
    return point_container()[0];

  // something went wront...
  return PointType();
}



class capacitor3d : public viennamini::device_template
{
private:
  typedef viennagrid::plc_3d_mesh                                       MeshType;
  typedef viennagrid::result_of::point<MeshType>::type                  MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type          MeshVertexHandleType;
  typedef viennagrid::result_of::vertex_id<MeshType>::type              MeshVertexIDType;
  typedef viennagrid::result_of::line_handle<MeshType>::type            MeshLineHandleType;
  typedef viennagrid::result_of::plc_handle<MeshType>::type             MeshPLCHandleType;

public:
  capacitor3d(std::string const& material_library_file, std::ostream& stream = std::cout)
    : viennamini::device_template(material_library_file, stream)
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
    mesher_->set_input( "cell_size", 0.01 );
    mesher_->set_input( "max_radius_edge_ratio", 1.5 );  // maximum radius edge ratio
    mesher_->set_input( "min_dihedral_angle", 0.17 );     // minimum dihedral angle in radiant, 0.17 are about 10 degrees
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

    device_  = viennamini::device_handle(new viennamini::device(this->stream()));
    config_  = viennamini::config_handle(new viennamini::config(this->stream()));

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
    viennamesh::result_of::parameter_handle< MeshType >::type   mesh_handle = viennamesh::make_parameter<MeshType>();
    MeshType & mesh = mesh_handle();

    std::vector<MeshVertexHandleType> vertices;
    std::vector<MeshLineHandleType>   linesFront;
    std::vector<MeshLineHandleType>   linesBack;
    std::vector<MeshLineHandleType>   linesTop;
    std::vector<MeshLineHandleType>   linesBottom;
    std::vector<MeshLineHandleType>   linesRight;
    std::vector<MeshLineHandleType>   linesLeft;
    std::vector<MeshLineHandleType>   linesIntRight;
    std::vector<MeshLineHandleType>   linesIntLeft;
    std::vector<MeshLineHandleType>   linesSegment2;
    std::vector<MeshLineHandleType>   linesSegment3;
    std::vector<MeshLineHandleType>   linesSegment4;

    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P1"]) ) ); // 0
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P2"]) ) ); // 1
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P3"]) ) ); // 2
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P4"]) ) ); // 3
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P5"]) ) ); // 4
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P6"]) ) ); // 5
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P7"]) ) ); // 6
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P8"]) ) ); // 7

    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI1"]) ) ); // 8
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI2"]) ) ); // 9
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI3"]) ) ); // 10
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI4"]) ) ); // 11
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI5"]) ) ); // 12
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI6"]) ) ); // 13
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI7"]) ) ); // 14
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PI8"]) ) ); // 15

    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC1"]) ) ); // 16
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC2"]) ) ); // 17
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC3"]) ) ); // 18
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC4"]) ) ); // 19
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC5"]) ) ); // 20
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC6"]) ) ); // 21

    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P11"]) ) ); // 22
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC41"]) ) ); // 23
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC31"]) ) ); // 24
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC11"]) ) ); // 25

    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["P71"]) ) ); // 26
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC51"]) ) ); // 27
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC21"]) ) ); // 28
    vertices.push_back( viennagrid::make_vertex( mesh, MeshPointType(geometry_properties()["PC61"]) ) ); // 29

    enum vertex_indices
    {
      p1,  // 0
      p2,  // 1
      p3,  // 2
      p4,  // 3
      p5,  // 4
      p6,  // 5
      p7,  // 6
      p8,  // 7
      pi1, // 8
      pi2, // 9
      pi3, // 10
      pi4, // 11
      pi5, // 12
      pi6, // 13
      pi7, // 14
      pi8, // 15
      pc1, // 16
      pc2, // 17
      pc3, // 18
      pc4, // 19
      pc5, // 20
      pc6, // 21
      p11, // 22
      pc41,// 23
      pc31,// 24
      pc11,// 25
      p71, // 26
      pc51,// 27
      pc21,// 28
      pc61 // 29
    };


    // We are using this map to keep track of all lines we use, this method is used by get_make_line which is used by make_quad_plc
    std::map< std::pair<MeshVertexIDType, MeshVertexIDType>, MeshLineHandleType > vertex_line_map;
    std::vector<MeshPLCHandleType> plcs;

    // ---------------------------------------------------------------------------
    //
    // Segment 1
    //

    // left
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p11], vertices[pc41], vertices[pc31], vertices[pc11] ) );
    }
    // front
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p11], vertices[p1], vertices[pc4], vertices[pc41] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc41], vertices[pc4], vertices[pc3], vertices[pc31] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc31], vertices[pc3], vertices[pc1], vertices[pc11] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p11], vertices[p1], vertices[pc1], vertices[pc11] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 2
    //

    // left (contains interface to Segment 1)
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p1],  vertices[pc1]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc1], vertices[pc3]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc3], vertices[pc4]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc4], vertices[p1]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc1], vertices[p4]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p4],  vertices[p8]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p8],  vertices[p5]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p5],  vertices[pc4]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }
    // front
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p1],  vertices[pc4]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc4], vertices[p5]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p5],  vertices[pi5]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi5],  vertices[pi1]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi1],  vertices[p1]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p5], vertices[pi5], vertices[pi8], vertices[p8] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p8], vertices[pi8], vertices[pi4], vertices[p4] ) );
    }
    // bottom
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p1],  vertices[pc1]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc1], vertices[p4]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p4],  vertices[pi4]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi4],  vertices[pi1]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi1],  vertices[p1]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }
    // right (interface to Segment 3)
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi5], vertices[pi8], vertices[pi4], vertices[pi1] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 3
    //

    // front
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi5], vertices[pi6], vertices[pi2], vertices[pi1] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi5], vertices[pi6], vertices[pi7], vertices[pi8] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi7], vertices[pi8], vertices[pi4], vertices[pi3] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi4], vertices[pi3], vertices[pi2], vertices[pi1] ) );
    }
    // right (interface to Segment 4)
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi6], vertices[pi7], vertices[pi3], vertices[pi2] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 4
    //

    // front
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi6], vertices[p6], vertices[p2], vertices[pi2] ) );
    }
    // top
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi6],  vertices[p6]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p6], vertices[pc6]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc6],  vertices[p7]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p7],  vertices[pi7]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi7],  vertices[pi6]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }
    // back
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi7],  vertices[p7]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p7], vertices[pc5]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc5],  vertices[p3]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p3],  vertices[pi3]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pi3],  vertices[pi7]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pi3], vertices[p3], vertices[p2], vertices[pi2] ) );
    }
    // right (contains interface to Segment 5)
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc6],  vertices[p7]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p7], vertices[pc5]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc5], vertices[pc2]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc2], vertices[pc6]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[pc5], vertices[p3]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p3],  vertices[p2]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p2],  vertices[p6]) );
      lines.push_back( get_make_line(mesh, vertex_line_map, vertices[p6],  vertices[pc6]) );

      plcs.push_back( viennagrid::make_plc(mesh, lines.begin(), lines.end()) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 5
    //

    // front
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc6], vertices[pc61], vertices[pc21], vertices[pc2] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc6], vertices[pc61], vertices[p71], vertices[p7] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[p7], vertices[p71], vertices[pc51], vertices[pc5] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc2], vertices[pc21], vertices[pc51], vertices[pc5] ) );
    }
    // right
    {
      plcs.push_back( make_quad_plc( mesh, vertex_line_map, vertices[pc61], vertices[p71], vertices[pc51], vertices[pc21] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Compute seed points for each segment
    //
    MeshPointType seed_point_segment_1 = get_seed_point( mesh, plcs.begin()+0, plcs.begin()+6 );
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

    MeshPointType seed_point_segment_2 = get_seed_point( mesh, plcs.begin()+5, plcs.begin()+11 );
//    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

    MeshPointType seed_point_segment_3 = get_seed_point( mesh, plcs.begin()+10, plcs.begin()+16 );
//    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

    MeshPointType seed_point_segment_4 = get_seed_point( mesh, plcs.begin()+15, plcs.begin()+21 );
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

    MeshPointType seed_point_segment_5 = get_seed_point( mesh, plcs.begin()+20, plcs.begin()+26 );
//    std::cout << "seed pnt 5: " << seed_point_segment_5 << std::endl;

    viennamesh::seed_point_3d_container seed_points;
    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) );
    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
    seed_points.push_back( std::make_pair(seed_point_segment_4, 4) );
    seed_points.push_back( std::make_pair(seed_point_segment_5, 5) );
    mesher_->set_input("seed_points", seed_points);

    // ---------------------------------------------------------------------------
    //
    // Generate the mesh
    //

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh_handle );

    mesher_->reference_output( "default", device_->get_segmesh_tetrahedral_3d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      stream() << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }

  void assign_segments()
  {
    device_->make_contact         (1);
    device_->set_name             (1, contact_a_);
    device_->set_material         (1, "Cu");

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


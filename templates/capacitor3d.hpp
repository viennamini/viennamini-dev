#ifndef VIENNAMINI_TEMPLATES_CAPACITOR3D_HPP
#define VIENNAMINI_TEMPLATES_CAPACITOR3D_HPP

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

class capacitor3d : public viennamini::device_template
{
private:
  typedef viennagrid::plc_3d_mesh                                       MeshType;
  typedef viennagrid::result_of::point<MeshType>::type                  MeshPointType;
  typedef viennagrid::result_of::vertex_handle<MeshType>::type          MeshVertexHandleType;
  typedef viennagrid::result_of::vertex_id<MeshType>::type              MeshVertexIDType;
  typedef viennagrid::result_of::line_handle<MeshType>::type            MeshLineHandleType;
  typedef viennagrid::result_of::plc_handle<MeshType>::type             MeshPLCHandleType;
  typedef viennagrid::result_of::segmentation<MeshType>::type           SegmentationType;
  typedef viennagrid::result_of::segment_handle<SegmentationType>::type SegmentHandleType;
  typedef viennagrid::segmented_mesh<MeshType, SegmentationType>        SegmentedMeshType;

public:
  capacitor3d(std::ostream& stream = std::cout)
    : viennamini::device_template(stream)
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

    problem_id_ = viennamini::id::laplace();

    device_handle_  = viennamini::device_handle(new viennamini::device(this->stream()));
    config_handle_  = viennamini::config_handle(new viennamini::config(this->stream()));
  }

  ~capacitor3d()
  {
  }

  /* virtual */
  void generate()
  {
    device_handle_->make_tetrahedral3d();

    this->generate_mesh();
    device_handle_->update_problem_description();
    this->assign_segments();
  }

private:
  void generate_mesh()
  {
    viennamesh::result_of::parameter_handle< SegmentedMeshType >::type geometry_handle = viennamesh::make_parameter<SegmentedMeshType>();
    MeshType          & geometry      = geometry_handle().mesh;
    SegmentationType  & segmentation  = geometry_handle().segmentation;

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

    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P1"]) ) ); // 0
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P2"]) ) ); // 1
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P3"]) ) ); // 2
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P4"]) ) ); // 3
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P5"]) ) ); // 4
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P6"]) ) ); // 5
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P7"]) ) ); // 6
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P8"]) ) ); // 7

    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI1"]) ) ); // 8
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI2"]) ) ); // 9
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI3"]) ) ); // 10
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI4"]) ) ); // 11
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI5"]) ) ); // 12
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI6"]) ) ); // 13
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI7"]) ) ); // 14
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PI8"]) ) ); // 15

    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC1"]) ) ); // 16
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC2"]) ) ); // 17
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC3"]) ) ); // 18
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC4"]) ) ); // 19
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC5"]) ) ); // 20
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC6"]) ) ); // 21

    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P11"]) ) ); // 22
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC41"]) ) ); // 23
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC31"]) ) ); // 24
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC11"]) ) ); // 25

    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["P71"]) ) ); // 26
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC51"]) ) ); // 27
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC21"]) ) ); // 28
    vertices.push_back( viennagrid::make_vertex( geometry, MeshPointType(geometry_properties()["PC61"]) ) ); // 29

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
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p11], vertices[pc41], vertices[pc31], vertices[pc11] ) );
    }
    // front
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p11], vertices[p1], vertices[pc4], vertices[pc41] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc41], vertices[pc4], vertices[pc3], vertices[pc31] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc31], vertices[pc3], vertices[pc1], vertices[pc11] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p11], vertices[p1], vertices[pc1], vertices[pc11] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 2
    //

    // left (contains interface to Segment 1)
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p1],  vertices[pc1]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc1], vertices[pc3]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc3], vertices[pc4]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc4], vertices[p1]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc1], vertices[p4]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p4],  vertices[p8]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p8],  vertices[p5]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p5],  vertices[pc4]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }
    // front
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p1],  vertices[pc4]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc4], vertices[p5]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p5],  vertices[pi5]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi5],  vertices[pi1]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi1],  vertices[p1]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p5], vertices[pi5], vertices[pi8], vertices[p8] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p8], vertices[pi8], vertices[pi4], vertices[p4] ) );
    }
    // bottom
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p1],  vertices[pc1]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc1], vertices[p4]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p4],  vertices[pi4]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi4],  vertices[pi1]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi1],  vertices[p1]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }
    // right (interface to Segment 3)
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi5], vertices[pi8], vertices[pi4], vertices[pi1] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 3
    //

    // front
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi5], vertices[pi6], vertices[pi2], vertices[pi1] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi5], vertices[pi6], vertices[pi7], vertices[pi8] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi7], vertices[pi8], vertices[pi4], vertices[pi3] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi4], vertices[pi3], vertices[pi2], vertices[pi1] ) );
    }
    // right (interface to Segment 4)
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi6], vertices[pi7], vertices[pi3], vertices[pi2] ) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 4
    //

    // front
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi6], vertices[p6], vertices[p2], vertices[pi2] ) );
    }
    // top
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi6],  vertices[p6]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p6], vertices[pc6]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc6],  vertices[p7]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p7],  vertices[pi7]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi7],  vertices[pi6]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }
    // back
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi7],  vertices[p7]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p7], vertices[pc5]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc5],  vertices[p3]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p3],  vertices[pi3]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pi3],  vertices[pi7]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pi3], vertices[p3], vertices[p2], vertices[pi2] ) );
    }
    // right (contains interface to Segment 5)
    {
      std::vector<MeshLineHandleType> lines;

      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc6],  vertices[p7]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p7], vertices[pc5]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc5], vertices[pc2]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc2], vertices[pc6]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[pc5], vertices[p3]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p3],  vertices[p2]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p2],  vertices[p6]) );
      lines.push_back( get_make_line(geometry, vertex_line_map, vertices[p6],  vertices[pc6]) );

      plcs.push_back( viennagrid::make_plc(geometry, lines.begin(), lines.end()) );
    }

    // ---------------------------------------------------------------------------
    //
    // Segment 5
    //

    // front
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc6], vertices[pc61], vertices[pc21], vertices[pc2] ) );
    }
    // top
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc6], vertices[pc61], vertices[p71], vertices[p7] ) );
    }
    // back
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[p7], vertices[p71], vertices[pc51], vertices[pc5] ) );
    }
    // bottom
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc2], vertices[pc21], vertices[pc51], vertices[pc5] ) );
    }
    // right
    {
      plcs.push_back( make_quad_plc( geometry, vertex_line_map, vertices[pc61], vertices[p71], vertices[pc51], vertices[pc21] ) );
    }

    // ---------------------------------------------------------------------------
    //
    //

    SegmentHandleType segment1 = segmentation.make_segment();
    viennagrid::add( segment1, plcs[0] );
    viennagrid::add( segment1, plcs[1] );
    viennagrid::add( segment1, plcs[2] );
    viennagrid::add( segment1, plcs[3] );
    viennagrid::add( segment1, plcs[4] );
    viennagrid::add( segment1, plcs[5] );

    SegmentHandleType segment2 = segmentation.make_segment();
    viennagrid::add( segment2, plcs[5] );
    viennagrid::add( segment2, plcs[6] );
    viennagrid::add( segment2, plcs[7] );
    viennagrid::add( segment2, plcs[8] );
    viennagrid::add( segment2, plcs[9] );
    viennagrid::add( segment2, plcs[10] );

    SegmentHandleType segment3 = segmentation.make_segment();
    viennagrid::add( segment3, plcs[10] );
    viennagrid::add( segment3, plcs[11] );
    viennagrid::add( segment3, plcs[12] );
    viennagrid::add( segment3, plcs[13] );
    viennagrid::add( segment3, plcs[14] );
    viennagrid::add( segment3, plcs[15] );

    SegmentHandleType segment4 = segmentation.make_segment();
    viennagrid::add( segment4, plcs[15] );
    viennagrid::add( segment4, plcs[16] );
    viennagrid::add( segment4, plcs[17] );
    viennagrid::add( segment4, plcs[18] );
    viennagrid::add( segment4, plcs[19] );
    viennagrid::add( segment4, plcs[20] );

    SegmentHandleType segment5 = segmentation.make_segment();
    viennagrid::add( segment5, plcs[20] );
    viennagrid::add( segment5, plcs[21] );
    viennagrid::add( segment5, plcs[22] );
    viennagrid::add( segment5, plcs[23] );
    viennagrid::add( segment5, plcs[24] );
    viennagrid::add( segment5, plcs[25] );

    // ---------------------------------------------------------------------------
    //
    // Generate the mesh
    //

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", geometry_handle );

    mesher_->reference_output( "default", device_handle_->get_segmesh_tetrahedral_3d() );
    if(!mesher_->run())
    {
      // TODO provide exception
      stream() << "Error: Meshing failed" << std::endl;
      exit(-1);
    }
  }

  void assign_segments()
  {
    device_handle_->make_contact         (0);
    device_handle_->set_name             (0, contact_a_);
    device_handle_->set_material         (0, "Cu");

    device_handle_->make_oxide           (1);
    device_handle_->set_name             (1, plate_a_);
    device_handle_->set_material         (1, "SiO2");

    device_handle_->make_semiconductor   (2);
    device_handle_->set_name             (2, insulator_);
    device_handle_->set_material         (2, "Si");

    device_handle_->make_oxide           (3);
    device_handle_->set_name             (3, plate_b_);
    device_handle_->set_material         (3, "SiO2");

    device_handle_->make_contact         (4);
    device_handle_->set_name             (4, contact_b_);
    device_handle_->set_material         (4, "Cu");
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


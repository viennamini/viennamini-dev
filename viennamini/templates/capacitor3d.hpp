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

    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P1"]) ) ); // 0
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P2"]) ) ); // 1
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P3"]) ) ); // 2
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P4"]) ) ); // 3
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P5"]) ) ); // 4
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P6"]) ) ); // 5
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P7"]) ) ); // 6
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P8"]) ) ); // 7

    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI1"]) ) ); // 8
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI2"]) ) ); // 9
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI3"]) ) ); // 10
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI4"]) ) ); // 11
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI5"]) ) ); // 12
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI6"]) ) ); // 13
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI7"]) ) ); // 14
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PI8"]) ) ); // 15

    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC1"]) ) ); // 16
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC2"]) ) ); // 17
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC3"]) ) ); // 18
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC4"]) ) ); // 19
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC5"]) ) ); // 20
    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC6"]) ) ); // 21

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
      pc6  // 21
    };

//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P11"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC11"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC31"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC41"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["P71"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC21"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC51"]) ) );
//    vertices.push_back( viennagrid::make_vertex( mesh(), MeshPointType(geometry_properties()["PC61"]) ) );

    // Front
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p1),    *(vertices.begin() + pc4)));  // 0: p1-pc4
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc4),   *(vertices.begin() + p5) ));  // 1: pc4-p5
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p5),    *(vertices.begin() + pi5)));  // 2: p5-pi5
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi5),   *(vertices.begin() + pi6)));  // 3: pi5-pi6
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi6),   *(vertices.begin() + p6) ));  // 4: pi6-p6
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p6),    *(vertices.begin() + p2) ));  // 5: p6-p2
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p2),    *(vertices.begin() + pi2) )); // 6: p2-pi2
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi2),   *(vertices.begin() + pi1) )); // 7: pi2-pi1
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi1),   *(vertices.begin() + p1) ));  // 8: pi1-p1
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi5),   *(vertices.begin() + pi1) )); // 9: pi5-pi1
    linesFront.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi6),   *(vertices.begin() + pi2) )); // 10: pi6-pi2

    // Back
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p4),  *(vertices.begin() + p8) )); // 0: p4-p8
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p8),  *(vertices.begin() + pi8))); // 1: p9-pi8
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi8), *(vertices.begin() + pi7))); // 2: pi8
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi7), *(vertices.begin() + p7) )); // 3: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p7),  *(vertices.begin() + pc5))); // 4: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc5), *(vertices.begin() + p3) )); // 5: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p3),  *(vertices.begin() + pi3))); // 6: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi3), *(vertices.begin() + pi4))); // 7: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi4), *(vertices.begin() + p4) )); // 8: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi8), *(vertices.begin() + pi4))); // 9: 
    linesBack.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi7), *(vertices.begin() + pi3))); // 10: 

    // Top
    linesTop.push_back( linesFront[2] ); // 0: p5-pi5
    linesTop.push_back( linesFront[3] ); // 1: pi5-pi6
    linesTop.push_back( linesFront[4] ); // 2: pi6-p6
    linesTop.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p6),   *(vertices.begin() + pc6))); // 3
    linesTop.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc6),  *(vertices.begin() + p7) )); // 4
    linesTop.push_back( linesBack[3] ); // 5: pi7-p7
    linesTop.push_back( linesBack[2] ); // 6: pi8-pi7
    linesTop.push_back( linesBack[1] ); // 7: p8-pi8
    linesTop.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p8),   *(vertices.begin() + p5) )); // 8
    linesTop.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi5),  *(vertices.begin() + pi8))); // 9
    linesTop.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi6),  *(vertices.begin() + pi7))); // 10

    // Bottom
    linesBottom.push_back( linesFront[8] ); // 0: p1-pi1
    linesBottom.push_back( linesFront[7] ); // 1: pi2-pi1
    linesBottom.push_back( linesFront[6] ); // 2: p2-pi2
    linesBottom.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p2),  *(vertices.begin() + p3)  )); // 3
    linesBottom.push_back( linesBack[6] ); // 4: p3-pi3
    linesBottom.push_back( linesBack[7] ); // 5: pi3-pi4
    linesBottom.push_back( linesBack[8] ); // 6: pi4-p4
    linesBottom.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + p4),  *(vertices.begin() + pc1) )); // 7
    linesBottom.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc1), *(vertices.begin() + p1)  )); // 8
    linesBottom.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi1),  *(vertices.begin() + pi4) )); // 9
    linesBottom.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi2),  *(vertices.begin() + pi3) )); // 10

    // Left
    linesLeft.push_back( linesFront[0]  ); // 0: p1-pc4
    linesLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc4),  *(vertices.begin() + pc3)  )); // 1
    linesLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc3),  *(vertices.begin() + pc1)  )); // 2
    linesLeft.push_back( linesBottom[8] ); // 3: pc1-p1
    linesLeft.push_back( linesBottom[7] ); // 4: p4-pc1
    linesLeft.push_back( linesBack[0]   ); // 5: p4-p8
    linesLeft.push_back( linesFront[1]  ); // 6: pc4-p5
    linesLeft.push_back( linesTop[8]    ); // 7: p8-p5

    // Right
    linesRight.push_back( linesFront[5] ); // 0: p6-p2
    linesRight.push_back( linesTop[3]   ); // 1: p6-pc6
    linesRight.push_back( linesTop[4]   ); // 2: pc6-p7
    linesRight.push_back( linesBack[4]  ); // 3: p7-pc5
    linesRight.push_back( linesBack[5]  ); // 4: pc5-p3
    linesRight.push_back( linesBottom[3]); // 5: p2-p3
    linesRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc6),  *(vertices.begin() + pc2)  )); // 6
    linesRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pc2),  *(vertices.begin() + pc5)  )); // 7

    // Interface Left
    linesIntLeft.push_back( linesFront[9] );
    linesIntLeft.push_back( linesTop[9] );
    linesIntLeft.push_back( linesBack[9] );
    linesIntLeft.push_back( linesBottom[9] );

    // Interface Right
    linesIntRight.push_back( linesFront[10] );
    linesIntRight.push_back( linesTop[10] );
    linesIntRight.push_back( linesBack[10] );
    linesIntRight.push_back( linesBottom[10] );

    // Setup PLCs
    typedef typename viennagrid::result_of::plc_handle<MeshType>::type  PLCHandleType;
    std::vector<PLCHandleType> plcs;

    plcs.push_back( viennagrid::make_plc(mesh(), linesFront.begin(),    linesFront.end()));   // 0
    plcs.push_back( viennagrid::make_plc(mesh(), linesTop.begin(),      linesTop.end()));     // 1
    plcs.push_back( viennagrid::make_plc(mesh(), linesBack.begin(),     linesBack.end()));    // 2
    plcs.push_back( viennagrid::make_plc(mesh(), linesBottom.begin(),   linesBottom.end()));  // 3
    plcs.push_back( viennagrid::make_plc(mesh(), linesRight.begin(),    linesRight.end()));   // 4
    plcs.push_back( viennagrid::make_plc(mesh(), linesLeft.begin(),     linesLeft.end()));    // 5
    plcs.push_back( viennagrid::make_plc(mesh(), linesIntRight.begin(), linesIntRight.end()));// 6
    plcs.push_back( viennagrid::make_plc(mesh(), linesIntLeft.begin(),  linesIntLeft.end())); // 7

    
    // Segment 2
    MeshPointType seed_segment_2;
    {
      MeshType temp_mesh;

      

//      { // front
//        std::vector<MeshLineHandleType> lines;
//        lines.push_back( linesFront[8] );
//        lines.push_back( linesFront[9] );
//        lines.push_back( linesFront[2] );
//        lines.push_back( linesFront[1] );
//        lines.push_back( linesFront[0] );
//        viennagrid::copy_element_handles(mesh(), lines.begin(), lines.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, lines.begin(), lines.end());
//      }
//      { // top
//        std::vector<MeshLineHandleType> lines;
//        lines.push_back( linesTop[0] );
//        lines.push_back( linesTop[9] );
//        lines.push_back( linesTop[7] );
//        lines.push_back( linesTop[8] );
//        viennagrid::copy_element_handles(mesh(), lines.begin(), lines.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, lines.begin(), lines.end());
//      }
//      { // back
//        std::vector<MeshLineHandleType> lines;
//        lines.push_back( linesBack[1] );
//        lines.push_back( linesBack[0] );
//        lines.push_back( linesBack[8] );
//        lines.push_back( linesBack[9] );
//        viennagrid::copy_element_handles(mesh(), lines.begin(), lines.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, lines.begin(), lines.end());
//      }
//      { // bottom
//        std::vector<MeshLineHandleType> lines;
//        lines.push_back( linesBottom[6] );
//        lines.push_back( linesBottom[9] );
//        lines.push_back( linesBottom[0] );
//        lines.push_back( linesBottom[8] );
//        lines.push_back( linesBottom[7] );
//        viennagrid::copy_element_handles(mesh(), lines.begin(), lines.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, lines.begin(), lines.end());
//      }
//      { // left
//        viennagrid::copy_element_handles(mesh(), linesLeft.begin(), linesLeft.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, linesLeft.begin(), linesLeft.end());
//      }
//      { // right
//        viennagrid::copy_element_handles(mesh(), linesIntLeft.begin(), linesIntLeft.end(), temp_mesh, 0.0 );
//        viennagrid::make_plc(temp_mesh, linesIntLeft.begin(), linesIntLeft.end());
//      }

      seed_segment_2 =  compute_seed_point(temp_mesh);
      std::cout << "seed pnt 2: " << seed_segment_2 << std::endl;
    }

//    //2nd from Far Left PLC : 4 connections
//    lines2ndLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi1),       *(vertices.begin() + pi5)  ));
//    lines2ndLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi5),   *(vertices.begin() + pi8)  ));
//    lines2ndLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi8),   *(vertices.begin() + pi4)  ));
//    lines2ndLeft.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi4),   *(vertices.begin() + pi1)  ));

//    //2nd from Far Right PLC : 4 connections
//    lines2ndRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi2),       *(vertices.begin() + pi6)  ));
//    lines2ndRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi6),   *(vertices.begin() + pi7)  ));
//    lines2ndRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi7),   *(vertices.begin() + pi3)  ));
//    lines2ndRight.push_back( viennagrid::make_line( mesh(), *(vertices.begin() + pi3),   *(vertices.begin() + pi2)  ));


//    PLCHandleType plc_handle1 = viennagrid::make_plc(mesh(), linesFront.begin(), linesFront.end());
//    PLCHandleType plc_handle2 = viennagrid::make_plc(mesh(), linesTop.begin(), linesTop.end());
//    PLCHandleType plc_handle3 = viennagrid::make_plc(mesh(), linesBack.begin(), linesBack.end());
//    PLCHandleType plc_handle4 = viennagrid::make_plc(mesh(), linesBottom.begin(), linesBottom.end());
//    PLCHandleType plc_handle5 = viennagrid::make_plc(mesh(), linesFarRight.begin(), linesFarRight.end());
//    PLCHandleType plc_handle6 = viennagrid::make_plc(mesh(), linesFarLeft.begin(), linesFarLeft.end());
//    PLCHandleType plc_handle7 = viennagrid::make_plc(mesh(), lines2ndLeft.begin(), lines2ndLeft.end());
//    PLCHandleType plc_handle8 = viennagrid::make_plc(mesh(), lines2ndRight.begin(), lines2ndRight.end());

//    // --------------------------------------------------
//    // Setup segment 1
//    //

//    // -- Lines

//    // bottom 
//    MeshLineHandleType line0 = viennagrid::make_line(mesh(), p11, p1);
//    MeshLineHandleType line1 = viennagrid::make_line(mesh(), p1, pc1);
//    MeshLineHandleType line2 = viennagrid::make_line(mesh(), pc1, pc11 );
//    MeshLineHandleType line3 = viennagrid::make_line(mesh(), pc11, p11);
//    
//    // top
//    MeshLineHandleType line4 = viennagrid::make_line(mesh(), pc41, pc4);
//    MeshLineHandleType line5 = viennagrid::make_line(mesh(), pc4,  pc3);
//    MeshLineHandleType line6 = viennagrid::make_line(mesh(), pc3,  pc31 );
//    MeshLineHandleType line7 = viennagrid::make_line(mesh(), pc31, pc41);
//    
//    // top-bottom connections
//    MeshLineHandleType line8 = viennagrid::make_line(mesh(),  pc41, p11);
//    MeshLineHandleType line9 = viennagrid::make_line(mesh(),  pc4,  p1);
//    MeshLineHandleType line10 = viennagrid::make_line(mesh(), pc3,  pc1 );
//    MeshLineHandleType line11 = viennagrid::make_line(mesh(), pc31, pc11);
//    
//    // -- PLCs
//    
//    // bottom 
//    lines.clear();
//    lines.push_back(line0);
//    lines.push_back(line1);
//    lines.push_back(line2);
//    lines.push_back(line3);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 0
//    
//    // top 
//    lines.clear();
//    lines.push_back(line4);
//    lines.push_back(line5);
//    lines.push_back(line6);
//    lines.push_back(line7);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 1

//    // front
//    lines.clear();
//    lines.push_back(line0);
//    lines.push_back(line9);
//    lines.push_back(line4);
//    lines.push_back(line8);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 2

//    // right
//    lines.clear();
//    lines.push_back(line1);
//    lines.push_back(line10);
//    lines.push_back(line5);
//    lines.push_back(line9);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 3


//    // back
//    lines.clear();
//    lines.push_back(line2);
//    lines.push_back(line10);
//    lines.push_back(line6);
//    lines.push_back(line11);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 4

//    // left
//    lines.clear();
//    lines.push_back(line3);
//    lines.push_back(line11);
//    lines.push_back(line7);
//    lines.push_back(line8);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 5

//    MeshPointType seed_point_segment_1 = compute_seed_point(mesh(), plcs.begin(), plcs.end());
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

//    // --------------------------------------------------
//    // Setup segment 1
//    //

//    // -- Lines

//    // bottom 
//    MeshLineHandleType line12 = viennagrid::make_line(mesh(), p1, pi1);
//    MeshLineHandleType line13 = viennagrid::make_line(mesh(), pi1, pi4);
//    MeshLineHandleType line14 = viennagrid::make_line(mesh(), pi4, p4);
//    MeshLineHandleType line15 = viennagrid::make_line(mesh(), p4, pc1);

//    // top
//    MeshLineHandleType line16 = viennagrid::make_line(mesh(), p5, pi5);
//    MeshLineHandleType line17 = viennagrid::make_line(mesh(), pi5, pi8);
//    MeshLineHandleType line18 = viennagrid::make_line(mesh(), pi8, p8);
//    MeshLineHandleType line19 = viennagrid::make_line(mesh(), p8, p5);

//    // top-bottom connections
//    MeshLineHandleType line20 = viennagrid::make_line(mesh(),  p5, pc4);
//    MeshLineHandleType line21 = viennagrid::make_line(mesh(),  pi5,  pi1);
//    MeshLineHandleType line22 = viennagrid::make_line(mesh(), pi8,  pi4 );
//    MeshLineHandleType line23 = viennagrid::make_line(mesh(), p8, p4);

//    // -- PLCs
//    
//    // bottom 
//    lines.clear();
//    lines.push_back(line12);
//    lines.push_back(line13);
//    lines.push_back(line14);
//    lines.push_back(line15);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 6

//    // top 
//    lines.clear();
//    lines.push_back(line16);
//    lines.push_back(line15);
//    lines.push_back(line14);
//    lines.push_back(line19);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 7

//    // front
//    lines.clear();
//    lines.push_back(line12);
//    lines.push_back(line21);
//    lines.push_back(line16);
//    lines.push_back(line20);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 7

//    // right
//    lines.clear();
//    lines.push_back(line13);
//    lines.push_back(line22);
//    lines.push_back(line17);
//    lines.push_back(line21);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 7

//    // back
//    lines.clear();
//    lines.push_back(line14);
//    lines.push_back(line22);
//    lines.push_back(line18);
//    lines.push_back(line23);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 7

//    // left
//    lines.clear();
//    lines.push_back(line15);
//    lines.push_back(line23);
//    lines.push_back(line19);
//    lines.push_back(line20);
//    plcs.push_back( viennagrid::make_plc( mesh(), lines.begin(), lines.end() ) ); // 7

//    MeshPointType seed_point_segment_1 = compute_seed_point(mesh(), plcs.begin()+5, plcs.end()+11);
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

//    MeshPointType seed_point_segment_1 = make_brick_and_seed_point(mesh, pc41, pc4, pc3, pc31, p11, p1, pc1, pc11);
//    std::cout << "seed pnt 1: " << seed_point_segment_1 << std::endl;

//    MeshPointType seed_point_segment_2 = make_brick_and_seed_point(mesh, p5, pi5, pi8, p8, p1, pi1, pi4, p4);
//    std::cout << "seed pnt 2: " << seed_point_segment_2 << std::endl;

//    MeshPointType seed_point_segment_3 = make_brick_and_seed_point(mesh, pi5, pi6, pi7, pi8, pi1, pi2, pi3, pi4);
//    std::cout << "seed pnt 3: " << seed_point_segment_3 << std::endl;

//    MeshPointType seed_point_segment_4 = make_brick_and_seed_point(mesh, pi6, p6, p7, pi7, pi2, p2, p3, pi3);
//    std::cout << "seed pnt 4: " << seed_point_segment_4 << std::endl;

//    MeshPointType seed_point_segment_5 = make_brick_and_seed_point(mesh, pc6, pc61, p71, p7, pc2, pc21, pc51, pc5);
//    std::cout << "seed pnt 5: " << seed_point_segment_5 << std::endl;

    // setting the created line geometry as input for the mesher
    mesher_->set_input( "default", mesh );

//    viennamesh::seed_point_3d_container seed_points;
//    seed_points.push_back( std::make_pair(seed_point_segment_1, 1) ); 
//    seed_points.push_back( std::make_pair(seed_point_segment_2, 2) );
////    seed_points.push_back( std::make_pair(seed_point_segment_3, 3) );
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
  
  
  MeshPointType compute_seed_point(MeshType const& mesh)
  {
    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
    seed_point_locator->set_input( "default", mesh);
    seed_point_locator->run();
    
    typedef viennamesh::result_of::point_container<MeshPointType>::type PointContainerType;
    viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );

//    std::cout << "found seedpoints: " << point_container().size() << std::endl;
//    for(PointContainerType::iterator iter = point_container().begin(); iter != point_container().end(); iter++)
//    {
//      std::cout << *iter << std::endl;
//    }
    
    if(point_container().size() != 1)
    {
      // TODO
      std::cout << "Error: More than one seed point computed" << std::endl;
      exit(-1);
    }
    
    return point_container().front();
  }
  
  
private:
//  template<typename MeshHandleT>
//  MeshPointType   make_brick_and_seed_point(MeshHandleT& mesh, 
//                  MeshVertexHandleType& p1, MeshVertexHandleType& p2, MeshVertexHandleType& p3, MeshVertexHandleType& p4, // top    - counter-clockwise
//                  MeshVertexHandleType& p5, MeshVertexHandleType& p6, MeshVertexHandleType& p7, MeshVertexHandleType& p8) // bottom - counter-clockwise
//  {
//    typedef typename viennagrid::result_of::plc_handle<MeshType>::type  PLCHandleType;
//    PLCHandleType      plcs[6];
//    MeshLineHandleType lines[12];

//    std::cout << "make brick --- " << std::endl;
//    std::cout << viennagrid::point(mesh(), p1) << std::endl;
//    std::cout << viennagrid::point(mesh(), p2) << std::endl;
//    std::cout << viennagrid::point(mesh(), p3) << std::endl;
//    std::cout << viennagrid::point(mesh(), p4) << std::endl;
//    std::cout << viennagrid::point(mesh(), p5) << std::endl;
//    std::cout << viennagrid::point(mesh(), p6) << std::endl;
//    std::cout << viennagrid::point(mesh(), p7) << std::endl;
//    std::cout << viennagrid::point(mesh(), p8) << std::endl;

//    // top
//    lines[0] = viennagrid::make_line(mesh(), p1, p2);
//    lines[1] = viennagrid::make_line(mesh(), p2, p3);
//    lines[2] = viennagrid::make_line(mesh(), p3, p4 );
//    lines[3] = viennagrid::make_line(mesh(), p4, p1);
//    
//    // bottom
//    lines[4] = viennagrid::make_line(mesh(), p5, p6);
//    lines[5] = viennagrid::make_line(mesh(), p6, p7);
//    lines[6] = viennagrid::make_line(mesh(), p7, p8 );
//    lines[7] = viennagrid::make_line(mesh(), p8, p5);
//    
//    // top-bottom connections
//    lines[8] = viennagrid::make_line(mesh(), p1, p5);
//    lines[9] = viennagrid::make_line(mesh(), p2, p6);
//    lines[10] = viennagrid::make_line(mesh(), p3, p7 );
//    lines[11] = viennagrid::make_line(mesh(), p4, p8);
//    
//    // top plc
//    plcs[0] = viennagrid::make_plc( mesh(), lines+0, lines+4 );
//    
//    // bottom plc
//    plcs[1] = viennagrid::make_plc( mesh(), lines+4, lines+8 );

//    // face 1
//    {
//      MeshLineHandleType cur_lines[4];
//      cur_lines[0] = lines[0];
//      cur_lines[1] = lines[9];
//      cur_lines[2] = lines[4];
//      cur_lines[3] = lines[8];
//      plcs[2] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
//    }

//    // face 2
//    {
//      MeshLineHandleType cur_lines[4];
//      cur_lines[0] = lines[1];
//      cur_lines[1] = lines[10];
//      cur_lines[2] = lines[5];
//      cur_lines[3] = lines[9];
//      plcs[3] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
//    }

//    // face 3
//    {
//      MeshLineHandleType cur_lines[4];
//      cur_lines[0] = lines[2];
//      cur_lines[1] = lines[10];
//      cur_lines[2] = lines[6];
//      cur_lines[3] = lines[11];
//      plcs[4] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
//    }

//    // face 4
//    {
//      MeshLineHandleType cur_lines[4];
//      cur_lines[0] = lines[3];
//      cur_lines[1] = lines[11];
//      cur_lines[2] = lines[7];
//      cur_lines[3] = lines[8];
//      plcs[5] = viennagrid::make_plc( mesh(), cur_lines+0, cur_lines+4 );
//    }
//    
////    MeshType temp_mesh;
////    viennagrid::copy_element_handles( mesh(), plcs+0, plcs+6, temp_mesh, 0.0 );
////    
////    viennamesh::algorithm_handle seed_point_locator( new viennamesh::seed_point_locator::algorithm() );
////    seed_point_locator->set_input( "default", temp_mesh);
////    seed_point_locator->run();
////    
////    typedef viennamesh::result_of::point_container<MeshPointType>::type PointContainerType;
////    viennamesh::result_of::parameter_handle<PointContainerType>::type point_container = seed_point_locator->get_output<PointContainerType>( "default" );
////    
////    if(point_container().size() != 1)
////    {
////      // TODO
////      std::cout << "Error: More than one seed point computed" << std::endl;
////      exit(-1);
////    }
////    
////    return point_container().front();
//  }



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


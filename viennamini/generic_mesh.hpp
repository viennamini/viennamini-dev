
#ifndef VIENNAMINI_GENERICMESH_HPP
#define VIENNAMINI_GENERICMESH_HPP

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

namespace viennamini 
{

/** @brief Exception for the case that the mesh type id is not supported */
class meshtype_not_supported_exception : public std::runtime_error {
public:
  meshtype_not_supported_exception()                        : std::runtime_error("") {}
  meshtype_not_supported_exception(std::string const & str) : std::runtime_error(str) {}
};

struct generic_mesh
{
  generic_mesh() : id(viennamini::mesh::none), segmesh_line_1d_ptr(NULL) { }

  ~generic_mesh()
  {
    if (id != viennamini::mesh::none)
    {
      if (id == viennamini::mesh::line_1d)
        delete segmesh_line_1d_ptr;
      else if (id == viennamini::mesh::triangular_2d)
        delete segmesh_triangular_2d_ptr;
      else if (id == viennamini::mesh::tetrahedral_3d)
        delete segmesh_tetrahedral_3d_ptr;

      id = viennamini::mesh::none;
      segmesh_line_1d_ptr = NULL;
    }
  }

  void generate(viennamini::mesh::mesh_ids   mesh_id)
  {
    if (mesh_id == viennamini::mesh::line_1d)
      segmesh_line_1d_ptr = new segmesh_line_1d;
    else if (mesh_id == viennamini::mesh::triangular_2d)
      segmesh_triangular_2d_ptr = new segmesh_triangular_2d;
    else if (mesh_id == viennamini::mesh::tetrahedral_3d)
      segmesh_tetrahedral_3d_ptr = new segmesh_tetrahedral_3d;
    else throw meshtype_not_supported_exception();

    id = mesh_id;
  }

  viennamini::mesh::mesh_ids id;
  union 
  {
    segmesh_line_1d         *  segmesh_line_1d_ptr;
    segmesh_triangular_2d   *  segmesh_triangular_2d_ptr;
    segmesh_tetrahedral_3d  *  segmesh_tetrahedral_3d_ptr;
  };
};

} // viennamini

#endif


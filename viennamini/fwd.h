#ifndef VIENNAMINI_FORWARDS_H
#define VIENNAMINI_FORWARDS_H

/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the ViennaFVM base directory
======================================================================= */

/** Forward declarations */


//#include "viennamini/device.hpp"

#include <iostream>

// ViennaData includes
#include "viennadata/api.hpp"

// ViennaGrid includes
#include "viennagrid/forwards.hpp"
#include "viennagrid/config/default_configs.hpp"

// ViennaMesh includes
#include "viennamesh/core/parameter.hpp"

// ViennaMaterials includes
//#include "viennamaterials/library.hpp"
//#include "viennamaterials/kernels/pugixml.hpp"


namespace viennamini
{
  //
  // TODO: Think about where to move the following keys.
  //

  struct permittivity_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(permittivity_key const & ) const { return false; }
  };

  struct mobility_electrons_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(mobility_electrons_key const & ) const { return false; }
  };

  struct mobility_holes_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(mobility_holes_key const & ) const { return false; }
  };

  struct builtin_potential_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(builtin_potential_key const & ) const { return false; }
  };

  // N_D
  struct donator_doping_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(donator_doping_key const & ) const { return false; }
  };

  // N_A
  struct acceptor_doping_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(acceptor_doping_key const & ) const { return false; }
  };

  struct oxide_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(oxide_key const & ) const { return false; }
  };

  struct semiconductor_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(semiconductor_key const & ) const { return false; }
  };

  struct contact_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(contact_key const & ) const { return false; }
  };

  struct material_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(material_key const & ) const { return false; }
  };

  struct potential_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(potential_key const & ) const { return false; }
  };

  struct electron_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(electron_key const & ) const { return false; }
  };

  struct hole_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(hole_key const & ) const { return false; }
  };


  // public typedefs
  //
  typedef ::viennadata::storage<>                                                                                 data_storage;

  typedef ::viennagrid::mesh< viennagrid::config::triangular_2d >                                                 mesh_triangular_2d;
  typedef ::viennagrid::mesh< viennagrid::config::tetrahedral_3d >                                                mesh_tetrahedral_3d;

  typedef ::viennagrid::result_of::segmentation<mesh_triangular_2d>::type                                         segmentation_triangular_2d;
  typedef ::viennagrid::result_of::segmentation<mesh_tetrahedral_3d>::type                                        segmentation_tetrahedral_3d;
  
  typedef ::viennamesh::MeshWrapper<viennamini::mesh_triangular_2d,  viennamini::segmentation_triangular_2d>      segmesh_triangular_2d;
  typedef ::viennamesh::MeshWrapper<viennamini::mesh_tetrahedral_3d, viennamini::segmentation_tetrahedral_3d>     segmesh_tetrahedral_3d;

  typedef ::boost::shared_ptr<segmesh_triangular_2d>                                                              segmesh_triangular_2d_ptr;
  typedef ::boost::shared_ptr<segmesh_tetrahedral_3d>                                                             segmesh_tetrahedral_3d_ptr;

  class triangular_2d  {};
  class tetrahedral_3d {};

  class null_mesh {};  


} // viennamini

#endif


#ifndef VIENNAMINI_DEVICE_HPP
#define VIENNAMINI_DEVICE_HPP

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

#include <map>

#include "viennamini/fwd.h"
#include "viennamini/device_segment_parameters.hpp"

#include "boost/variant.hpp"


namespace viennamini 
{

struct device
{
public:
  // [JW] note that the first type in a boost::variant must be a default constructible object
  //
  typedef boost::variant<viennamini::null_segmentation, SegmentationTriangular2DType, SegmentationTetrahedral3DType>    GenericSegmentationType;
  typedef boost::variant<viennamini::null_mesh,         MeshTriangular2DType,         MeshTetrahedral3DType>            GenericMeshType;
  typedef double                                                                                                        NumericType;
  typedef viennamini::segment_parameters<NumericType>                                                                   SegmentParametersType;
  typedef std::map<int, SegmentParametersType >                                                                         MeshParametersType;

  typedef GenericSegmentationType                                                                                       generic_segmentation_type;
  typedef GenericMeshType                                                                                               generic_mesh_type;
  typedef NumericType                                                                                                   numeric_type;
  typedef SegmentParametersType                                                                                         segment_parameters_type;
  typedef MeshParametersType                                                                                            mesh_parameters_type;

  device(viennamini::StorageType& storage);

  void make_triangular2d();

  void make_tetrahedral3d();
  
  bool is_triangular2d();

  bool is_tetrahedral3d();
  
  std::string& name(int id);
  std::string& material(int id);

  void make_contact(int id);
  void make_oxide(int id);
  void make_semiconductor(int id);
  
  NumericType& contact_potential(int id);
  NumericType& workfunction(int id);
  NumericType& NA_max(int id);
  NumericType& ND_max(int id);

  GenericMeshType         & generic_mesh();
  
  GenericSegmentationType & generic_segmentation();

  SegmentParametersType   & segment_parameters(int id);
  
  viennamini::StorageType& storage();

private:
  viennamini::StorageType&  storage_;
  GenericMeshType           generic_mesh_;
  GenericSegmentationType   generic_segmentation_;
  MeshParametersType        mesh_parameters_;
};


} // viennamini

#endif


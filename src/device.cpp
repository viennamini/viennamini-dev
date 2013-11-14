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


#include "viennamini/device.hpp"


namespace viennamini
{

device::device(viennamini::StorageType& storage) : storage_(storage) {};

void device::make_triangular2d()
{
  generic_mesh_         = viennamini::MeshTriangular2DType();
  generic_segmentation_ = viennamini::SegmentationTriangular2DType(boost::get<viennamini::MeshTriangular2DType>(generic_mesh_));
}

void device::make_tetrahedral3d()
{
  generic_mesh_         = viennamini::MeshTetrahedral3DType();
  generic_segmentation_ = viennamini::SegmentationTetrahedral3DType(boost::get<viennamini::MeshTetrahedral3DType>(generic_mesh_)); 
}

bool device::is_triangular2d()
{
  viennamini::MeshTriangular2DType         * mesh_pnt = boost::get<viennamini::MeshTriangular2DType>        ( &generic_mesh_ );
  viennamini::SegmentationTriangular2DType * seg_pnt  = boost::get<viennamini::SegmentationTriangular2DType>( &generic_segmentation_ );
  if(mesh_pnt && seg_pnt) return true;
  else    return false;
}

bool device::is_tetrahedral3d()
{
  viennamini::MeshTetrahedral3DType         * mesh_pnt = boost::get<viennamini::MeshTetrahedral3DType>        ( &generic_mesh_ );
  viennamini::SegmentationTetrahedral3DType * seg_pnt  = boost::get<viennamini::SegmentationTetrahedral3DType>( &generic_segmentation_ );
  if(mesh_pnt && seg_pnt) return true;
  else    return false;
}

std::string& device::name(int id)
{
  return mesh_parameters_[id].name();
}

std::string& device::material(int id)
{
  return mesh_parameters_[id].material();
}

void device::make_contact(int id)
{
  mesh_parameters_[id].is_contact() = true;
}

void device::make_oxide(int id)
{
  mesh_parameters_[id].is_oxide() = true;
}

void device::make_semiconductor(int id)
{
  mesh_parameters_[id].is_semiconductor() = true;
}

device::NumericType& device::NA_max(int id)
{
  return mesh_parameters_[id].NA_max();
}

device::NumericType& device::ND_max(int id)
{
  return mesh_parameters_[id].ND_max();
}

device::GenericMeshType& device::generic_mesh()
{ 
  return generic_mesh_; 
}

device::GenericSegmentationType& device::generic_segmentation() 
{ 
  return generic_segmentation_; 
}

device::SegmentParametersType& device::segment_parameters(int id)
{
  return mesh_parameters_[id];
}

viennamini::StorageType& device::storage()
{
  return storage_;
}

} // viennamini


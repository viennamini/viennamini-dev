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

//#include "viennamesh/algorithm/file_reader.hpp"
#include "viennagrid/io/netgen_reader.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/scale.hpp"

#include "viennamini/detect_interfaces.hpp"

namespace viennamini
{

device::device(viennamini::data_storage& storage) : storage_(storage) 
{
}

void device::make_triangular2d()
{
  generic_mesh_ = segmesh_triangular_2d_ptr(new segmesh_triangular_2d_ptr::element_type);
}

void device::make_tetrahedral3d()
{
  generic_mesh_ = segmesh_tetrahedral_3d_ptr(new segmesh_tetrahedral_3d_ptr::element_type);
}

bool device::is_triangular2d()
{
  segmesh_triangular_2d_ptr pnt = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
  if(pnt) return true;
  else    return false;
}

bool device::is_tetrahedral3d()
{
  segmesh_tetrahedral_3d_ptr pnt = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
  if(pnt) return true;
  else    return false;
}

segmesh_triangular_2d& device::get_segmesh_triangular_2d()
{
  return *boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
}

segmesh_tetrahedral_3d& device::get_segmesh_tetrahedral_3d()
{
  return *boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
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

device::NumericType& device::contact_potential(int id)
{
  return mesh_parameters_[id].contact_potential();
}

device::NumericType& device::workfunction(int id)
{
  return mesh_parameters_[id].workfunction();
}

device::NumericType& device::NA_max(int id)
{
  return mesh_parameters_[id].NA_max();
}

device::NumericType& device::ND_max(int id)
{
  return mesh_parameters_[id].ND_max();
}

void device::update()
{
  oxide_segments_indices_.clear();
  contact_segments_indices_.clear();
  semiconductor_segments_indices_.clear();
  contactSemiconductorInterfaces_.clear();
  contactOxideInterfaces_.clear();

  for(MeshParametersType::iterator siter = mesh_parameters_.begin();
      siter != mesh_parameters_.end(); siter++)
  {
    if(siter->second.is_oxide()) oxide_segments_indices_.push_back(siter->first);
    else
    if(siter->second.is_contact()) contact_segments_indices_.push_back(siter->first);
    else
    if(siter->second.is_semiconductor()) semiconductor_segments_indices_.push_back(siter->first);
  }

  viennamini::detect_interfaces(*this, contactSemiconductorInterfaces_, contactOxideInterfaces_);
}

device::GenericMeshType& device::generic_mesh()
{ 
  return generic_mesh_; 
}

device::SegmentParametersType& device::segment_parameters(int id)
{
  return mesh_parameters_[id];
}

viennamini::data_storage& device::storage()
{
  return storage_;
}

void device::read(std::string const& filename, viennamini::triangular_2d const&)
{
//  viennamesh::AlgorithmHandle reader = viennamesh::AlgorithmHandle( new viennamesh::FileReader() );
//  reader->set_input( "filename", filename );
////  reader->set_output();
////  if(this->is_triangular2d())
////    reader->get_output( "default" )->convert_to( boost::get<segmesh_triangular_2d_ptr>(generic_mesh_));

////  segmesh_triangular_2d_ptr temp (new segmesh_triangular_2d_ptr::element_type);
////  reader->get_output( "default" )->convert_to( temp );

//  //boost::get<segmesh_triangular_2d_ptr>(generic_mesh_) = 
//  segmesh_triangular_2d_ptr temp = reader->get_output( "default" )->get_converted<segmesh_triangular_2d_ptr::element_type>();

//  reader->run();
//  viennamini::io::read_vtk(mydevice, "../external/ViennaDeviceCollection/nin2d/nin2d.mesh", viennagrid::config::triangular_2d());

  this->make_triangular2d();
  segmesh_triangular_2d_ptr segmesh = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
  viennagrid::io::netgen_reader   reader;
  reader(segmesh->mesh, segmesh->segmentation, filename);  
}

void device::read(std::string const& filename, viennamini::tetrahedral_3d const&)
{
  this->make_tetrahedral3d();
  segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
  viennagrid::io::netgen_reader reader;
  reader(segmesh->mesh, segmesh->segmentation, filename);  
}

void device::write(std::string const& filename)
{
  if(this->is_triangular2d())
  {
    segmesh_triangular_2d_ptr segmesh = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
    viennagrid::io::vtk_writer<mesh_triangular_2d> vtk_writer;
    vtk_writer(segmesh->mesh, segmesh->segmentation, filename);
  }
  else if(this->is_tetrahedral3d())
  {
    segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
    viennagrid::io::vtk_writer<mesh_tetrahedral_3d> vtk_writer;
    vtk_writer(segmesh->mesh, segmesh->segmentation, filename);
  }
}

void device::scale(numeric_type factor)
{
  if(this->is_triangular2d())
  {
    segmesh_triangular_2d_ptr segmesh = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
    viennagrid::scale(segmesh->mesh, factor);  
  }
  else if(this->is_tetrahedral3d())
  {
    segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
    viennagrid::scale(segmesh->mesh, factor);  
  }
}

device::IndicesType&   device::contact_segments_indices()
{
  return contact_segments_indices_;
}

device::IndicesType&   device::oxide_segments_indices()
{
  return oxide_segments_indices_;
}

device::IndicesType&   device::semiconductor_segments_indices()
{
  return semiconductor_segments_indices_;
}

} // viennamini


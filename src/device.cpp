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

#include "viennafvm/boundary.hpp"

#include "viennamini/detect_interfaces.hpp"
#include "viennamini/constants.hpp"

namespace viennamini
{

struct is_line_1d_visitor : public boost::static_visitor<bool>
{
  bool operator()(segmesh_line_1d_ptr & ptr) const { return true; }

  template<typename T>
  bool operator()(T & ptr) const { return false; }
};

struct is_triangle_2d_visitor : public boost::static_visitor<bool>
{
  bool operator()(segmesh_triangular_2d_ptr & ptr) const { return true; }

  template<typename T>
  bool operator()(T & ptr) const { return false; }
};

struct is_tetrahedral_3d_visitor : public boost::static_visitor<bool>
{
  bool operator()(segmesh_tetrahedral_3d_ptr & ptr) const { return true; }

  template<typename T>
  bool operator()(T & ptr) const { return false; }
};

device::device() :   matlib_(new viennamini::material_library())
{
}

void device::make_line1d()
{
  generic_mesh_                = segmesh_line_1d_ptr(new segmesh_line_1d_ptr::element_type);
  generic_problem_description_ = problem_description_line_1d(get_segmesh_line_1d().mesh);
}

void device::make_triangular2d()
{
  generic_mesh_                = segmesh_triangular_2d_ptr(new segmesh_triangular_2d_ptr::element_type);
  generic_problem_description_ = problem_description_triangular_2d(get_segmesh_triangular_2d().mesh);
}

void device::make_tetrahedral3d()
{
  generic_mesh_                 = segmesh_tetrahedral_3d_ptr(new segmesh_tetrahedral_3d_ptr::element_type);
  generic_problem_description_  = problem_description_tetrahedral_3d(get_segmesh_tetrahedral_3d().mesh);
}

bool device::is_line1d()
{
  return boost::apply_visitor(is_line_1d_visitor(), generic_mesh_);
}

bool device::is_triangular2d()
{
  return boost::apply_visitor(is_triangle_2d_visitor(), generic_mesh_);
}

bool device::is_tetrahedral3d()
{
  return boost::apply_visitor(is_tetrahedral_3d_visitor(), generic_mesh_);
}

segmesh_line_1d& device::get_segmesh_line_1d()
{
  return *boost::get<segmesh_line_1d_ptr>(generic_mesh_);
}

segmesh_triangular_2d& device::get_segmesh_triangular_2d()
{
  return *boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
}

segmesh_tetrahedral_3d& device::get_segmesh_tetrahedral_3d()
{
  return *boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
}

problem_description_line_1d&  device::get_problem_description_line_1d()
{
  return boost::get<problem_description_line_1d>(generic_problem_description_);
}

problem_description_triangular_2d&  device::get_problem_description_triangular_2d()
{
  return boost::get<problem_description_triangular_2d>(generic_problem_description_);
}

problem_description_tetrahedral_3d& device::get_problem_description_tetrahedral_3d()
{
  return boost::get<problem_description_tetrahedral_3d>(generic_problem_description_);
}

void device::make_contact(int segment_index)
{
  segment_roles_[segment_index] = role::contact;
}

void device::make_oxide(int segment_index)
{
  segment_roles_[segment_index] = role::oxide;
}

void device::make_semiconductor(int segment_index)
{
  segment_roles_[segment_index] = role::semiconductor;
}

bool device::is_contact(int segment_index)
{
  return segment_roles_[segment_index] == role::contact;
}

bool device::is_oxide(int segment_index)
{
  return segment_roles_[segment_index] == role::oxide;
}

bool device::is_semiconductor(int segment_index)
{
  return segment_roles_[segment_index] == role::semiconductor;
}

bool device::is_contact_at_oxide(int segment_index)
{
  return !(contact_oxide_interfaces_.find(segment_index) == contact_oxide_interfaces_.end());
}

bool device::is_contact_at_semiconductor(int segment_index)
{
  return !(contact_semiconductor_interfaces_.find(segment_index) == contact_semiconductor_interfaces_.end());
}

std::size_t device::get_adjacent_semiconductor_segment_for_contact(int segment_index)
{
  return contact_semiconductor_interfaces_[segment_index];
}

std::size_t device::get_adjacent_oxide_segment_for_contact(int segment_index)
{
  return contact_oxide_interfaces_[segment_index];
}

void device::update()
{
  oxide_segments_indices_.clear();
  contact_segments_indices_.clear();
  semiconductor_segments_indices_.clear();
  contact_semiconductor_interfaces_.clear();
  contact_oxide_interfaces_.clear();

  for(SegmentRolesType::iterator siter = segment_roles_.begin();
      siter != segment_roles_.end(); siter++)
  {
    if(siter->second == role::oxide)         oxide_segments_indices_.push_back(siter->first);
    else
    if(siter->second == role::contact)       contact_segments_indices_.push_back(siter->first);
    else
    if(siter->second == role::semiconductor) semiconductor_segments_indices_.push_back(siter->first);
  }

  // identify contact-semiconductor/oxide interfaces 
  //
  viennamini::detect_interfaces(*this, contact_semiconductor_interfaces_, contact_oxide_interfaces_);
  
  // transfer permittivity from the semiconductor/oxide segments to 
  // adjacent contact segments
  //
  for(IndicesType::iterator contact_iter = contact_segments_indices_.begin();
      contact_iter != contact_segments_indices_.end(); contact_iter++)
  {
    if(this->is_contact_at_oxide(*contact_iter))
    {
      std::size_t adjacent_segment_index    = this->get_adjacent_oxide_segment_for_contact(*contact_iter);
      std::string adjacent_segment_material = this->get_material(adjacent_segment_index);
      
      if(this->material_library()()->has_parameter(adjacent_segment_material, viennamini::material::relative_permittivity()))
        this->set_relative_permittivity(*contact_iter, this->material_library()()->get_parameter_value(adjacent_segment_material, viennamini::material::relative_permittivity()) );
    }
    else
    if(this->is_contact_at_semiconductor(*contact_iter))
    {
      std::size_t adjacent_segment_index    = this->get_adjacent_semiconductor_segment_for_contact(*contact_iter);
      std::string adjacent_segment_material = this->get_material(adjacent_segment_index);
      
      if(this->material_library()()->has_parameter(adjacent_segment_material, viennamini::material::relative_permittivity()))
        this->set_relative_permittivity(*contact_iter, this->material_library()()->get_parameter_value(adjacent_segment_material, viennamini::material::relative_permittivity()) );
    }
  }
}

device::GenericMeshType& device::generic_mesh()
{ 
  return generic_mesh_; 
}

device::GenericProblemDescriptionType & device::generic_problem_description()
{
  return generic_problem_description_;
}

viennamini::material_library& device::material_library()
{ 
  return *matlib_;
}

void device::read(std::string const& filename, viennamini::line_1d const&)
{
  this->make_line1d();
  segmesh_line_1d_ptr segmesh = boost::get<segmesh_line_1d_ptr>(generic_mesh_);
  viennagrid::io::netgen_reader   reader;
  reader(segmesh->mesh, segmesh->segmentation, filename);
  this->update_problem_description();
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
  this->update_problem_description();
}

void device::read(std::string const& filename, viennamini::tetrahedral_3d const&)
{
  this->make_tetrahedral3d();
  segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
  viennagrid::io::netgen_reader reader;
  reader(segmesh->mesh, segmesh->segmentation, filename);
  this->update_problem_description();
}

void device::read_material_library(std::string const& filename)
{
  matlib_->read(filename);
}

void device::write(std::string const& filename)
{
  if(this->is_line1d())
  {
    segmesh_line_1d_ptr segmesh = boost::get<segmesh_line_1d_ptr>(generic_mesh_);
    viennagrid::io::vtk_writer<mesh_line_1d> vtk_writer;
    vtk_writer(segmesh->mesh, segmesh->segmentation, filename);
  }
  else 
  if(this->is_triangular2d())
  {
    segmesh_triangular_2d_ptr segmesh = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
    viennagrid::io::vtk_writer<mesh_triangular_2d> vtk_writer;
    vtk_writer(segmesh->mesh, segmesh->segmentation, filename);
  }
  else 
  if(this->is_tetrahedral3d())
  {
    segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
    viennagrid::io::vtk_writer<mesh_tetrahedral_3d> vtk_writer;
    vtk_writer(segmesh->mesh, segmesh->segmentation, filename);
  }
  else throw device_not_supported_exception("at: device::write()");
}


void device::scale(viennamini::numeric factor)
{
  if(this->is_line1d())
  {
    segmesh_line_1d_ptr segmesh = boost::get<segmesh_line_1d_ptr>(generic_mesh_);
    viennagrid::scale(segmesh->mesh, factor);  
  }
  else
  if(this->is_triangular2d())
  {
    segmesh_triangular_2d_ptr segmesh = boost::get<segmesh_triangular_2d_ptr>(generic_mesh_);
    viennagrid::scale(segmesh->mesh, factor);  
  }
  else 
  if(this->is_tetrahedral3d())
  {
    segmesh_tetrahedral_3d_ptr segmesh = boost::get<segmesh_tetrahedral_3d_ptr>(generic_mesh_);
    viennagrid::scale(segmesh->mesh, factor);  
  }
  else throw device_not_supported_exception("at: device::scale()");
}

void device::set_name(int segment_index, std::string const& new_name)
{
  segment_names_[segment_index] = new_name;
}

void device::set_material(int segment_index, std::string const& new_material)
{
  segment_materials_[segment_index] = new_material;
  
  // set the relative permittivity of this segment, as we know the material now
  // extract the data from the material database
  //
  if(this->material_library()()->has_parameter(new_material, viennamini::material::relative_permittivity()))
    this->set_relative_permittivity(segment_index, this->material_library()()->get_parameter_value(new_material, viennamini::material::relative_permittivity()) );
#ifdef VIENNAMINI_VERBOSE
  else 
  {
    if(!is_contact(segment_index)) // for contacts, setting the epsr doesn't make sense
      std::cout << "Device: material \"" << new_material << "\" does not have the parameter \"" << viennamini::material::relative_permittivity() << "\" - skipping .." << std::endl;
  }
#endif
}

std::string device::get_name(int segment_index)
{
  return segment_names_[segment_index];
}

std::string device::get_material(int segment_index)
{
  return segment_materials_[segment_index];
}

void device::set_contact_potential(int segment_index, viennamini::numeric potential)
{
  if(this->is_line1d())
  {
    std::cout << "setting contact potential of " << potential << " for segment " << segment_index << std::endl;
    typedef problem_description_line_1d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::potential());
    viennafvm::set_dirichlet_boundary(quan, this->get_segmesh_line_1d().segmentation(segment_index), potential);
  }
  else
  if(this->is_triangular2d())
  {
    typedef problem_description_triangular_2d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::potential());
    viennafvm::set_dirichlet_boundary(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), potential);
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::potential());
    viennafvm::set_dirichlet_boundary(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), potential);
  }
  else throw device_not_supported_exception("at: device::set_contact_potential()");
}

void device::add_contact_workfunction(int segment_index, viennamini::numeric potential)
{
  if(this->is_line1d())
  {
    typedef problem_description_line_1d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::potential());
    viennafvm::addto_dirichlet_boundary(quan, this->get_segmesh_line_1d().segmentation(segment_index), potential);
  }
  else
  if(this->is_triangular2d())
  {
    typedef problem_description_triangular_2d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::potential());
    viennafvm::addto_dirichlet_boundary(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), potential);
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::potential());
    viennafvm::addto_dirichlet_boundary(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), potential);
  }
  else throw device_not_supported_exception("at: device::add_contact_workfunction()");
}

void device::set_relative_permittivity(int segment_index, viennamini::numeric epsr)
{
  if(this->is_line1d())
  {
    typedef problem_description_line_1d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::permittivity());
    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), epsr * viennamini::eps0::val());
  }
  else
  if(this->is_triangular2d())
  {
    typedef problem_description_triangular_2d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::permittivity());
    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), epsr * viennamini::eps0::val());
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::permittivity());
    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), epsr * viennamini::eps0::val());
  }
  else throw device_not_supported_exception("at: device::set_relative_permittivity()");
}

void device::set_acceptor_doping(int segment_index, viennamini::numeric NA)
{
  segment_acceptor_doping_[segment_index] = NA;

  if(this->is_line1d())
  {
    typedef problem_description_line_1d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::acceptor_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), NA);
  }
  else
  if(this->is_triangular2d())
  {
    typedef problem_description_triangular_2d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::acceptor_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), NA);
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::acceptor_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), NA);
  }
  else throw device_not_supported_exception("at: device::set_acceptor_doping()");
}

void device::set_donator_doping(int segment_index, viennamini::numeric ND)
{
  segment_donator_doping_[segment_index] = ND;

  if(this->is_line1d())
  {
    typedef problem_description_line_1d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::donator_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), ND);
  }
  else
  if(this->is_triangular2d())
  {
    typedef problem_description_triangular_2d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::donator_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), ND);
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::donator_doping());
    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), ND);
  }
  else throw device_not_supported_exception("at: device::set_donator_doping()");
}

void device::set_recombination(int segment_index, recombination::recombination_ids id)
{
  segment_recombinations_[segment_index] = id;
}

recombination::recombination_ids device::get_recombination(int segment_index)
{
  return segment_recombinations_[segment_index];
}

viennamini::numeric device::get_acceptor_doping(int segment_index)
{
  return segment_acceptor_doping_[segment_index];
}

viennamini::numeric device::get_donator_doping(int segment_index)
{
  return segment_donator_doping_[segment_index];
}

void device::update_problem_description()
{
  if(this->is_line1d())
  {
    get_problem_description_line_1d().clear_quantities();
    get_problem_description_line_1d().add_quantity(viennamini::id::potential());
    get_problem_description_line_1d().add_quantity(viennamini::id::permittivity());
    get_problem_description_line_1d().add_quantity(viennamini::id::donator_doping());
    get_problem_description_line_1d().add_quantity(viennamini::id::acceptor_doping());
  }
  else
  if(this->is_triangular2d())
  {
    get_problem_description_triangular_2d().clear_quantities();
    get_problem_description_triangular_2d().add_quantity(viennamini::id::potential());
    get_problem_description_triangular_2d().add_quantity(viennamini::id::permittivity());
    get_problem_description_triangular_2d().add_quantity(viennamini::id::donator_doping());
    get_problem_description_triangular_2d().add_quantity(viennamini::id::acceptor_doping());
  }
  else
  if(this->is_tetrahedral3d())
  {
    get_problem_description_tetrahedral_3d().clear_quantities();
    get_problem_description_tetrahedral_3d().add_quantity(viennamini::id::potential());
    get_problem_description_tetrahedral_3d().add_quantity(viennamini::id::permittivity());
    get_problem_description_tetrahedral_3d().add_quantity(viennamini::id::donator_doping());
    get_problem_description_tetrahedral_3d().add_quantity(viennamini::id::acceptor_doping());
  }
  else throw device_not_supported_exception("at: device::update_problem_description()");
}

std::string& device::description()
{
  return description_;
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


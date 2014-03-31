/* =======================================================================
   Copyright (c) 2011-2013, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */




//#include "viennamesh/algorithm/file_reader.hpp"
#include "viennagrid/io/netgen_reader.hpp"
#include "viennagrid/io/vtk_reader.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/scale.hpp"

#include "viennafvm/boundary.hpp"

#include "viennamini/device.hpp"
#include "viennamini/detect_interfaces.hpp"
#include "viennamini/constants.hpp"
#include "viennamini/physics.hpp"
#include "viennamini/utils/file_extension.hpp"
#include "viennamini/utils/is_zero.hpp"
#include "viennamini/utils/convert.hpp"
#include "viennamini/material_accessors.hpp"

namespace viennamini
{

device::device(std::ostream& stream) : matlib_(), stream_(stream)
{
}

void device::make_line1d()
{
  mesh_.generate(viennamini::mesh::line_1d);
}

void device::make_triangular2d()
{
  mesh_.generate(viennamini::mesh::triangular_2d);
}

void device::make_tetrahedral3d()
{
  mesh_.generate(viennamini::mesh::tetrahedral_3d);
}

bool device::is_line1d()
{
  return mesh_.id == viennamini::mesh::line_1d;
}

bool device::is_triangular2d()
{
  return mesh_.id == viennamini::mesh::triangular_2d;
}

bool device::is_tetrahedral3d()
{
  return mesh_.id == viennamini::mesh::tetrahedral_3d;
}

segmesh_line_1d& device::get_segmesh_line_1d()
{
  return *mesh_.segmesh_line_1d_ptr;
}

segmesh_triangular_2d& device::get_segmesh_triangular_2d()
{
  return *mesh_.segmesh_triangular_2d_ptr;
}

segmesh_tetrahedral_3d& device::get_segmesh_tetrahedral_3d()
{
  return *mesh_.segmesh_tetrahedral_3d_ptr;
}

void device::make(viennamini::role::segment_role_ids role, int segment_index, std::string const& name, std::string const& material)
{
    segment_roles_[segment_index] = role;
    segment_names_[segment_index] = name;
    this->set_material(segment_index, material);
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

viennamini::role::segment_role_ids device::get_segment_role(int segment_index)
{
  return segment_roles_[segment_index];
}

std::string device::get_segment_role_string(int segment_index)
{
    if(this->is_oxide(segment_index))
        return viennamini::id::oxide();
    else
    if(this->is_semiconductor(segment_index))
        return viennamini::id::semiconductor();
    else
    if(this->is_contact(segment_index))
        return viennamini::id::contact();
    else throw device_exception("Segment "+viennamini::convert<std::string>(segment_index)+" does not have a valid role!");
}


int device::get_adjacent_semiconductor_segment_for_contact(int segment_index)
{
  return contact_semiconductor_interfaces_[segment_index];
}

int device::get_adjacent_oxide_segment_for_contact(int segment_index)
{
  return contact_oxide_interfaces_[segment_index];
}

int device::get_adjacent_segment_for_contact(int segment_index)
{
  if(is_contact_at_oxide(segment_index)) return get_adjacent_oxide_segment_for_contact(segment_index);
  else
  if(is_contact_at_semiconductor(segment_index)) return get_adjacent_semiconductor_segment_for_contact(segment_index);
  else throw device_exception("Could not determine adjacent segment for contact " +
    viennamini::convert<std::string>(segment_index) + "\"" + get_name(segment_index) + "\"" );
}


void device::update()
{
  namespace vmat = viennamaterials;

  oxide_segments_indices_.clear();
  contact_segments_indices_.clear();
  semiconductor_segments_indices_.clear();
  contact_semiconductor_interfaces_.clear();
  contact_oxide_interfaces_.clear();
  segment_indices_.clear();

  for(SegmentRolesType::iterator siter = segment_roles_.begin();
      siter != segment_roles_.end(); siter++)
  {
    if(siter->second == role::oxide)         oxide_segments_indices_.push_back(siter->first);
    else
    if(siter->second == role::contact)       contact_segments_indices_.push_back(siter->first);
    else
    if(siter->second == role::semiconductor) semiconductor_segments_indices_.push_back(siter->first);
    else
    if(siter->second == role::none)
      throw unassigned_segment_role_exception(
        "Segment "+viennamini::convert<std::string>(siter->first)+
        " lacks a segment role, such as 'oxide'.");
    else
      throw unassigned_segment_role_exception(
        "Segment "+viennamini::convert<std::string>(siter->first)+
        " lacks a segment role, such as 'oxide'.");
  }

  // record the segment indices
  //
  if(this->is_line1d())
  {
    for(segmentation_line_1d::iterator sit = get_segmesh_line_1d().segmentation.begin();
        sit != get_segmesh_line_1d().segmentation.end(); ++sit)
    {
      segment_indices_.push_back(sit->id());
    }
  }
  else
  if(this->is_triangular2d())
  {
    for(segmentation_triangular_2d::iterator sit = get_segmesh_triangular_2d().segmentation.begin();
        sit != get_segmesh_triangular_2d().segmentation.end(); ++sit)
    {
      segment_indices_.push_back(sit->id());
    }
  }
  else
  if(this->is_tetrahedral3d())
  {
    for(segmentation_tetrahedral_3d::iterator sit = get_segmesh_tetrahedral_3d().segmentation.begin();
        sit != get_segmesh_tetrahedral_3d().segmentation.end(); ++sit)
    {
      segment_indices_.push_back(sit->id());
    }
  }

  // identify contact-semiconductor/oxide interfaces
  //
  viennamini::detect_interfaces(*this, contact_semiconductor_interfaces_, contact_oxide_interfaces_);


  // finalize quantities
  //
  if(this->is_line1d())
  {
    for(segmentation_line_1d::iterator sit = get_segmesh_line_1d().segmentation.begin();
        sit != get_segmesh_line_1d().segmentation.end(); ++sit)
    {
      if(this->has_quantity(viennamini::id::temperature(), sit->id()))
      {
        this->set_quantity(viennamini::id::thermal_potential(), sit->id(), viennamini::thermal_potential_functor(this->get_quantity(viennamini::id::temperature(), sit->id())));
      }
    }
  }
  else
  if(this->is_triangular2d())
  {
    for(segmentation_triangular_2d::iterator sit = get_segmesh_triangular_2d().segmentation.begin();
        sit != get_segmesh_triangular_2d().segmentation.end(); ++sit)
    {
      if(this->has_quantity(viennamini::id::temperature(), sit->id()))
      {
        this->set_quantity(viennamini::id::thermal_potential(), sit->id(), viennamini::thermal_potential_functor(this->get_quantity(viennamini::id::temperature(), sit->id())));
      }
    }
  }
  else
  if(this->is_tetrahedral3d())
  {
    for(segmentation_tetrahedral_3d::iterator sit = get_segmesh_tetrahedral_3d().segmentation.begin();
        sit != get_segmesh_tetrahedral_3d().segmentation.end(); ++sit)
    {
      if(this->has_quantity(viennamini::id::temperature(), sit->id()))
      {
        this->set_quantity(viennamini::id::thermal_potential(), sit->id(), viennamini::thermal_potential_functor(this->get_quantity(viennamini::id::temperature(), sit->id())));
      }
    }
  }


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


      numeric epsr_value    = this->material_library()->query_value(
        vmat::make_query(vmat::make_entry(this->matlib_material() , adjacent_segment_material),
                         vmat::make_entry(this->matlib_parameter(), material::relative_permittivity()),
                         vmat::make_entry(this->matlib_data()     , material::value()))
      );
//      std::cout << "transferring permittivity from oxide to contact " << adjacent_segment_index << " " << adjacent_segment_material << " " << epsr_value << std::endl;
      this->set_quantity(viennamini::id::relative_permittivity(), *contact_iter, epsr_value);
    }
    else
    if(this->is_contact_at_semiconductor(*contact_iter))
    {
      std::size_t adjacent_segment_index    = this->get_adjacent_semiconductor_segment_for_contact(*contact_iter);
      std::string adjacent_segment_material = this->get_material(adjacent_segment_index);

      numeric epsr_value    = this->material_library()->query_value(
        vmat::make_query(vmat::make_entry(this->matlib_material() , adjacent_segment_material),
                         vmat::make_entry(this->matlib_parameter(), material::relative_permittivity()),
                         vmat::make_entry(this->matlib_data()     , material::value()))
      );
//      std::cout << "transferring permittivity from semiconductor to contact " << adjacent_segment_index << " " << adjacent_segment_material << " " << epsr_value << std::endl;
      this->set_quantity(viennamini::id::relative_permittivity(), *contact_iter, epsr_value);
    }
  }
}

device::GenericMeshType& device::mesh()
{
  return mesh_;
}

material_library_handle & device::material_library()
{
  if(!matlib_.get()) throw device_lacks_material_library("");
  return matlib_;
}

void device::read(std::string const& filename, viennamini::line_1d const&)
{
  this->make_line1d();

  std::string extension = viennamini::file_extension(filename);
  if(extension == "mesh")
  {
    viennagrid::io::netgen_reader reader;
    reader(get_segmesh_line_1d().mesh, get_segmesh_line_1d().segmentation, filename);
  }
  else if( (extension == "pvd") || (extension == "vtu") )
  {
    viennagrid::io::vtk_reader<mesh_line_1d, segmentation_line_1d> reader;
    reader(get_segmesh_line_1d().mesh, get_segmesh_line_1d().segmentation, filename);
  }
  else
    throw unknown_mesh_file_exception(filename);
}

void device::read(std::string const& filename, viennamini::triangular_2d const&)
{
  this->make_triangular2d();

  std::string extension = viennamini::file_extension(filename);
  if(extension == "mesh")
  {
    viennagrid::io::netgen_reader reader;
    reader(get_segmesh_triangular_2d().mesh, get_segmesh_triangular_2d().segmentation, filename);
  }
  else if( (extension == "pvd") || (extension == "vtu") )
  {
    viennagrid::io::vtk_reader<mesh_triangular_2d, segmentation_triangular_2d> reader;
    reader(get_segmesh_triangular_2d().mesh, get_segmesh_triangular_2d().segmentation, filename);
  }
  else
    throw unknown_mesh_file_exception(filename);
}

void device::read(std::string const& filename, viennamini::tetrahedral_3d const&)
{
  this->make_tetrahedral3d();

  std::string extension = viennamini::file_extension(filename);
  if(extension == "mesh")
  {
    viennagrid::io::netgen_reader reader;
    reader(get_segmesh_tetrahedral_3d().mesh, get_segmesh_tetrahedral_3d().segmentation, filename);
  }
  else if( (extension == "pvd") || (extension == "vtu") )
  {
    viennagrid::io::vtk_reader<mesh_tetrahedral_3d, segmentation_tetrahedral_3d> reader;
    reader(get_segmesh_tetrahedral_3d().mesh, get_segmesh_tetrahedral_3d().segmentation, filename);
  }
  else
    throw unknown_mesh_file_exception(filename);
}

void device::set_material_library(material_library_handle& matlib)
{
  matlib_.reset();
  matlib_ = matlib;
  matlib_material_  = matlib_->register_accessor(new viennamini::xpath_material_accessor);
  matlib_model_     = matlib_->register_accessor(new viennamini::xpath_model_accessor);
  matlib_parameter_ = matlib_->register_accessor(new viennamini::xpath_parameter_accessor);
  matlib_data_      = matlib_->register_accessor(new viennamini::xpath_data_accessor);
}

void device::read_material_library(std::string const& filename)
{
  matlib_.reset();
  std::string extension = viennamini::file_extension(filename);
  if(extension == "xml")
  {
    matlib_           = material_library_handle(new viennamaterials::pugixml(filename));
    matlib_material_  = matlib_->register_accessor(new viennamini::xpath_material_accessor);
    matlib_model_     = matlib_->register_accessor(new viennamini::xpath_model_accessor);
    matlib_parameter_ = matlib_->register_accessor(new viennamini::xpath_parameter_accessor);
    matlib_data_      = matlib_->register_accessor(new viennamini::xpath_data_accessor);
  }
  else
  {
    throw unknown_material_library_file_exception(filename);
  }
}

void device::write(std::string const& filename)
{
  if(this->is_line1d())
  {
    viennagrid::io::vtk_writer<mesh_line_1d> vtk_writer;
    vtk_writer(get_segmesh_line_1d().mesh, get_segmesh_line_1d().segmentation, filename);
  }
  else
  if(this->is_triangular2d())
  {
    viennagrid::io::vtk_writer<mesh_triangular_2d> vtk_writer;
    vtk_writer(get_segmesh_triangular_2d().mesh, get_segmesh_triangular_2d().segmentation, filename);
  }
  else
  if(this->is_tetrahedral3d())
  {
    viennagrid::io::vtk_writer<mesh_tetrahedral_3d> vtk_writer;
    vtk_writer(get_segmesh_tetrahedral_3d().mesh, get_segmesh_tetrahedral_3d().segmentation, filename);
  }
  else throw device_not_supported_exception("at: device::write()");
}


void device::scale(viennamini::numeric factor)
{
  if(this->is_line1d())
  {
    viennagrid::scale(get_segmesh_line_1d().mesh, factor);
  }
  else
  if(this->is_triangular2d())
  {
    viennagrid::scale(get_segmesh_triangular_2d().mesh, factor);
  }
  else
  if(this->is_tetrahedral3d())
  {
    viennagrid::scale(get_segmesh_tetrahedral_3d().mesh, factor);
  }
  else throw device_not_supported_exception("at: device::scale()");
}

void device::set_material(int segment_index, std::string const& new_material)
{
  // store the material key for this segment
  //
  segment_materials_[segment_index] = new_material;

  // if the segment is a contact, setting material parameters does not make sense, at least for now
  //
  if(this->is_contact(segment_index)) return;

  namespace vmat = viennamaterials;

  // set the permittivity for this segment. this should be done for oxides and semiconductors
  //
  numeric epsr_value    = this->material_library()->query_value(
    vmat::make_query(vmat::make_entry(this->matlib_material() , new_material),
                     vmat::make_entry(this->matlib_parameter(), material::relative_permittivity()),
                     vmat::make_entry(this->matlib_data()     , material::value()))
  );
  this->set_quantity(viennamini::id::relative_permittivity(), segment_index, epsr_value);

  // the following quantities are only available for semiconductors
  //
  if(this->is_semiconductor(segment_index))
  {
      // set the electron mobility for this segment
      //
      numeric mu_n_0_value    = this->material_library()->query_value(
        vmat::make_query(vmat::make_entry(this->matlib_material() , new_material),
                         vmat::make_entry(this->matlib_parameter(), material::base_electron_mobility()),
                         vmat::make_entry(this->matlib_data()     , material::value()))
      );
      this->set_quantity(viennamini::id::electron_mobility(), segment_index, mu_n_0_value);

      // set the hole mobility for this segment
      //
      numeric mu_p_0_value    = this->material_library()->query_value(
        vmat::make_query(vmat::make_entry(this->matlib_material() , new_material),
                         vmat::make_entry(this->matlib_parameter(), material::base_hole_mobility()),
                         vmat::make_entry(this->matlib_data()     , material::value()))
      );
      this->set_quantity(viennamini::id::hole_mobility(), segment_index, mu_p_0_value);

      numeric ni_value        = this->material_library()->query_value(
        vmat::make_query(vmat::make_entry(this->matlib_material() , new_material),
                         vmat::make_entry(this->matlib_parameter(), material::intrinsic_carrier_concentration()),
                         vmat::make_entry(this->matlib_data()     , material::value()))
      );
      this->set_quantity(viennamini::id::intrinsic_carrier(), segment_index, ni_value);
  }
}

std::string device::get_name(int segment_index)
{
  return segment_names_[segment_index];
}

std::string device::get_material(int segment_index)
{
  return segment_materials_[segment_index];
}

void device::set_quantity(std::string const& quantity_name, viennamini::numeric const& value)
{
  if(this->is_line1d())
  {
    for(segmentation_line_1d::iterator sit = get_segmesh_line_1d().segmentation.begin();
        sit != get_segmesh_line_1d().segmentation.end(); ++sit)
    {
      this->set_quantity(quantity_name, (*sit).id(), value);
    }
  }
  else
  if(this->is_triangular2d())
  {
    for(segmentation_triangular_2d::iterator sit = get_segmesh_triangular_2d().segmentation.begin();
        sit != get_segmesh_triangular_2d().segmentation.end(); ++sit)
    {
      this->set_quantity(quantity_name, (*sit).id(), value);
    }
  }
  else
  if(this->is_tetrahedral3d())
  {
    for(segmentation_tetrahedral_3d::iterator sit = get_segmesh_tetrahedral_3d().segmentation.begin();
        sit != get_segmesh_tetrahedral_3d().segmentation.end(); ++sit)
    {
      this->set_quantity(quantity_name, (*sit).id(), value);
    }
  }
}

void device::set_quantity(std::string const& quantity_name, int segment_index, viennamini::numeric const& value)
{
  quantity_database_[quantity_name][segment_index].clear();

  if(this->is_line1d())
  {
    typedef viennagrid::result_of::cell_range<segment_line_1d>::type      CellOnSegmentRange;
    typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type     CellOnSegmentIterator;
    typedef viennagrid::result_of::cell<mesh_line_1d>::type               CellType;

    CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_line_1d().segmentation[segment_index]);
    for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
      quantity_database_[quantity_name][segment_index][(*cit).id().get()] = value;
  }
  else
  if(this->is_triangular2d())
  {
    typedef viennagrid::result_of::cell_range<segment_triangular_2d>::type  CellOnSegmentRange;
    typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type       CellOnSegmentIterator;
    typedef viennagrid::result_of::cell<mesh_triangular_2d>::type           CellType;

    CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_triangular_2d().segmentation[segment_index]);
    for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
      quantity_database_[quantity_name][segment_index][(*cit).id().get()] = value;
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef viennagrid::result_of::cell_range<segment_tetrahedral_3d>::type   CellOnSegmentRange;
    typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type         CellOnSegmentIterator;
    typedef viennagrid::result_of::cell<mesh_tetrahedral_3d>::type            CellType;

    CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_tetrahedral_3d().segmentation[segment_index]);
    for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
      quantity_database_[quantity_name][segment_index][(*cit).id().get()] = value;
  }
}

void device::set_quantity(std::string const& quantity_name, viennamini::sparse_values const& values)
{
  quantity_database_[quantity_name].clear();

  if(this->is_line1d())
  {
    typedef viennagrid::result_of::cell_range<mesh_line_1d>::type      CellRange;
    typedef viennagrid::result_of::cell<mesh_line_1d>::type            CellType;
    typedef viennagrid::result_of::segment_id_range<segmentation_line_1d,CellType >::type SegmentIDRange;

    CellRange cells = viennagrid::elements<CellType>(get_segmesh_line_1d().mesh);

    for(viennamini::sparse_values::const_iterator iter = values.begin(); iter != values.end(); iter++)
    {
      // iter->first:  cell index
      // iter->second: value for the indexed cell

      SegmentIDRange range = viennagrid::segment_ids(get_segmesh_line_1d().segmentation, cells[iter->first]);
      for(SegmentIDRange::iterator riter = range.begin(); riter != range.end(); riter++)
      {
        quantity_database_[quantity_name][*riter][iter->first] = iter->second;
      }
    }
  }
  else
  if(this->is_triangular2d())
  {
    typedef viennagrid::result_of::cell_range<mesh_triangular_2d>::type      CellRange;
    typedef viennagrid::result_of::cell<mesh_triangular_2d>::type            CellType;
    typedef viennagrid::result_of::segment_id_range<segmentation_triangular_2d,CellType >::type SegmentIDRange;

    CellRange cells = viennagrid::elements<CellType>(get_segmesh_triangular_2d().mesh);

    for(viennamini::sparse_values::const_iterator iter = values.begin(); iter != values.end(); iter++)
    {
      // iter->first:  cell index
      // iter->second: value for the indexed cell

      SegmentIDRange range = viennagrid::segment_ids(get_segmesh_triangular_2d().segmentation, cells[iter->first]);
      for(SegmentIDRange::iterator riter = range.begin(); riter != range.end(); riter++)
      {
        quantity_database_[quantity_name][*riter][iter->first] = iter->second;
      }
    }
  }
  else
  if(this->is_tetrahedral3d())
  {
    typedef viennagrid::result_of::cell_range<mesh_tetrahedral_3d>::type      CellRange;
    typedef viennagrid::result_of::cell<mesh_tetrahedral_3d>::type            CellType;
    typedef viennagrid::result_of::segment_id_range<segmentation_tetrahedral_3d,CellType >::type SegmentIDRange;

    CellRange cells = viennagrid::elements<CellType>(get_segmesh_tetrahedral_3d().mesh);

    for(viennamini::sparse_values::const_iterator iter = values.begin(); iter != values.end(); iter++)
    {
      // iter->first:  cell index
      // iter->second: value for the indexed cell

      SegmentIDRange range = viennagrid::segment_ids(get_segmesh_tetrahedral_3d().segmentation, cells[iter->first]);
      for(SegmentIDRange::iterator riter = range.begin(); riter != range.end(); riter++)
      {
        quantity_database_[quantity_name][*riter][iter->first] = iter->second;
      }
    }
  }
}

void device::set_quantity(std::string const& quantity_name, int segment_index, viennamini::sparse_values const& values)
{
  quantity_database_[quantity_name][segment_index].clear();
  quantity_database_[quantity_name][segment_index].insert(values.begin(), values.end());
}

viennamini::sparse_values device::get_quantity(std::string const& quantity_name, int segment_index)
{
  return quantity_database_[quantity_name][segment_index];
}

viennamini::numeric device::get_quantity(std::string const& quantity_name, int segment_index, std::size_t cell_index)
{
  return quantity_database_[quantity_name][segment_index][cell_index];
}

void device::set_contact (std::string const& quantity_name, int segment_index, viennamini::numeric   const& value)
{
  contact_database_[quantity_name][segment_index] = value;
}

viennamini::numeric device::get_contact (std::string const& quantity_name, int segment_index)
{
//  if(!(this->has_contact(quantity_name, segment_index)))
//    throw device_exception("Device contact quantity \""+quantity_name+
//          "\" is not available on segment "+viennamini::convert<std::string>()(segment_index));
  return contact_database_[quantity_name][segment_index];
}

bool device::has_contact (std::string const& quantity_name, int segment_index)
{
  if(contact_database_.find(quantity_name) != contact_database_.end())
  {
    if(contact_database_[quantity_name].find(segment_index) != contact_database_[quantity_name].end())
      return true;
    else return false;
  }
  else return false;
}

bool device::has_quantity(std::string const& quantity_name, int segment_index)
{
  if(quantity_database_.find(quantity_name) != quantity_database_.end())
  {
    if(quantity_database_[quantity_name].find(segment_index) != quantity_database_[quantity_name].end())
    {
      if(this->is_line1d())
      {
        typedef viennagrid::result_of::cell_range<segment_line_1d>::type            CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type           CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_line_1d>::type                     CellType;
        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_line_1d().segmentation[segment_index]);
        for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
        {
          if(quantity_database_[quantity_name][segment_index].find(cit->id().get()) == quantity_database_[quantity_name][segment_index].end())
            return false;
        }
        return true;
      }
      else
      if(this->is_triangular2d())
      {
        typedef viennagrid::result_of::cell_range<segment_triangular_2d>::type      CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type           CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_triangular_2d>::type               CellType;
        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_triangular_2d().segmentation[segment_index]);
        for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
        {
          if(quantity_database_[quantity_name][segment_index].find(cit->id().get()) == quantity_database_[quantity_name][segment_index].end())
            return false;
        }
        return true;
      }
      else
      if(this->is_tetrahedral3d())
      {
        typedef viennagrid::result_of::cell_range<segment_tetrahedral_3d>::type      CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type           CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_tetrahedral_3d>::type               CellType;
        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_tetrahedral_3d().segmentation[segment_index]);
        for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
        {
          if(quantity_database_[quantity_name][segment_index].find(cit->id().get()) == quantity_database_[quantity_name][segment_index].end())
            return false;
        }
        return true;
      }
      else return false;
    }
    else return false;
  }
  else return false;
}

void device::set_recombination(int segment_index, recombination::recombination_ids id)
{
  segment_recombinations_[segment_index] = id;
}

recombination::recombination_ids device::get_recombination(int segment_index)
{
  return segment_recombinations_[segment_index];
}

std::string& device::description()
{
  return description_;
}

device::IndicesType&   device::segment_indices()
{
  return segment_indices_;
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

viennamaterials::accessor_handle& device::matlib_material()
{
  return matlib_material_;
}

viennamaterials::accessor_handle& device::matlib_model()
{
  return matlib_model_;
}


viennamaterials::accessor_handle& device::matlib_parameter()
{
  return matlib_parameter_;
}

viennamaterials::accessor_handle& device::matlib_data()
{
  return matlib_data_;
}

std::ostream& device::stream()
{
  return stream_;
}

// Private member functions

//void device::distribute_permittivity(std::size_t segment_index)
//{
//  if(this->is_line1d())
//  {
//    typedef problem_description_line_1d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::permittivity());
//    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), segment_permittivity_[segment_index]);
//  }
//  else
//  if(this->is_triangular2d())
//  {
//    typedef problem_description_triangular_2d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::permittivity());
//    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), segment_permittivity_[segment_index]);
//  }
//  else
//  if(this->is_tetrahedral3d())
//  {
//    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::permittivity());
//    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), segment_permittivity_[segment_index]);
//  }
//  else throw device_not_supported_exception("at: device::set_permittivity()");
//}

//void device::distribute_acceptor_doping(std::size_t segment_index)
//{
//  if(this->is_line1d())
//  {
//    typedef problem_description_line_1d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::acceptor_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), segment_acceptor_doping_[segment_index]);
//  }
//  else
//  if(this->is_triangular2d())
//  {
//    typedef problem_description_triangular_2d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::acceptor_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), segment_acceptor_doping_[segment_index]);
//  }
//  else
//  if(this->is_tetrahedral3d())
//  {
//    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::acceptor_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), segment_acceptor_doping_[segment_index]);
//  }
//  else throw device_not_supported_exception("at: device::set_acceptor_doping()");
//}

//void device::distribute_donator_doping(std::size_t segment_index)
//{
//  if(this->is_line1d())
//  {
//    typedef problem_description_line_1d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_line_1d().get_quantity(viennamini::id::donator_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_line_1d().segmentation(segment_index), segment_donator_doping_[segment_index]);
//  }
//  else
//  if(this->is_triangular2d())
//  {
//    typedef problem_description_triangular_2d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_triangular_2d().get_quantity(viennamini::id::donator_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_triangular_2d().segmentation(segment_index), segment_donator_doping_[segment_index]);
//  }
//  else
//  if(this->is_tetrahedral3d())
//  {
//    typedef problem_description_tetrahedral_3d::quantity_type  QuantityType;
//    QuantityType & quan = this->get_problem_description_tetrahedral_3d().get_quantity(viennamini::id::donator_doping());
//    viennafvm::set_initial_value(quan, this->get_segmesh_tetrahedral_3d().segmentation(segment_index), segment_donator_doping_[segment_index]);
//  }
//  else throw device_not_supported_exception("at: device::set_donator_doping()");
//}



} // viennamini



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

   license:    see file LICENSE in the base directory
======================================================================= */

#include <map>
#include <vector>

#include "viennamaterials/pugixml.hpp"

#include "viennamini/forwards.h"
#include "viennamini/generic_mesh.hpp"

//#include "boost/variant.hpp"

namespace viennamini
{

  /** @brief Exception for the case that the device type is not supported */
  class device_not_supported_exception : public std::runtime_error {
  public:
    device_not_supported_exception()                        : std::runtime_error("") {}
    device_not_supported_exception(std::string const & str) : std::runtime_error(str) {}
  };

  /** @brief Exception for the case that the input material library xml file is not supported */
  class unknown_material_library_file_exception : public std::runtime_error {
  public:
    unknown_material_library_file_exception(std::string const & str) : std::runtime_error(str) {}
  };

  /** @brief Exception for the case that the input mesh file is not supported */
  class unknown_mesh_file_exception : public std::runtime_error {
  public:
    unknown_mesh_file_exception(std::string const & str) : std::runtime_error(str) {}
  };

  /** @brief Exception for the case that the eps value is zero, indicating a problem with accessing the material database */
  class eps_is_zero_exception : public std::runtime_error {
  public:
    eps_is_zero_exception(std::string const & str) : std::runtime_error(str) {}
  };

  class segment_material_information_missing : public std::runtime_error {
  public:
    segment_material_information_missing(std::string const & str) : std::runtime_error(str) {}
  };

  class unassigned_segment_role_exception : public std::runtime_error {
  public:
    unassigned_segment_role_exception(std::string const & str) : std::runtime_error(str) {}
  };

  class device_lacks_material_library : public std::runtime_error {
  public:
    device_lacks_material_library(std::string const & str) : std::runtime_error(str) {}
  };

  class device
  {
  private:
    typedef std::map<std::size_t, role::segment_role_ids>                                         SegmentRolesType;
    typedef std::map<std::size_t, recombination::recombination_ids>                               SegmentRecombinationsType;
    typedef std::map<std::size_t, mobility::mobility_ids>                                         SegmentMobilityType;

  public:
    typedef generic_mesh    GenericMeshType;

    typedef std::vector<std::size_t>                                                       IndicesType;
    typedef std::map<std::size_t, std::size_t>                                             IndexMapType;
    typedef std::map<std::size_t, std::string>                                             IndexKeysType;
    typedef std::map<std::size_t, viennamini::numeric>                                     IndexValuesType;

    typedef GenericMeshType                                                                generic_mesh_type;
    typedef IndicesType                                                                    indices_type;
    typedef IndexMapType                                                                   index_map_type;
    typedef IndexKeysType                                                                  index_keys_type;

    device(std::ostream& stream = std::cout);

    void make_line1d();
    void make_triangular2d();
    void make_tetrahedral3d();

    bool is_line1d();
    bool is_triangular2d();
    bool is_tetrahedral3d();

    segmesh_line_1d&        get_segmesh_line_1d();
    segmesh_triangular_2d&  get_segmesh_triangular_2d();
    segmesh_tetrahedral_3d& get_segmesh_tetrahedral_3d();

    void make_neutral       (int segment_index);
    void make_contact       (int segment_index);
    void make_oxide         (int segment_index);
    void make_semiconductor (int segment_index);

    bool is_contact                 (int segment_index);
    bool is_contact_at_oxide        (int segment_index);
    bool is_contact_at_semiconductor(int segment_index);
    bool is_oxide                   (int segment_index);
    bool is_semiconductor           (int segment_index);

    viennamini::numeric&  temperature(); // TODO make this also a cell quantity

    std::size_t get_adjacent_semiconductor_segment_for_contact(int segment_index);
    std::size_t get_adjacent_oxide_segment_for_contact        (int segment_index);

    void update();

    GenericMeshType               & mesh();
    material_library_handle       & material_library();


    void read(std::string const& filename, viennamini::line_1d const&);
    void read(std::string const& filename, viennamini::triangular_2d const&);
    void read(std::string const& filename, viennamini::tetrahedral_3d const&);

    void read_material_library(std::string const& filename);
    void set_material_library(material_library_handle& matlib);

    void write(std::string const& filename);

    void scale(viennamini::numeric factor);

    void set_name                 (int segment_index, std::string const& new_name);
    void set_material             (int segment_index, std::string const& new_material);
    std::string get_name          (int segment_index);
    std::string get_material      (int segment_index);

    void set_permittivity         (int segment_index, viennamini::numeric epsr);
    void set_acceptor_doping      (int segment_index, viennamini::numeric NA);
    void set_donator_doping       (int segment_index, viennamini::numeric ND);

    viennamini::numeric get_permittivity(int segment_index);
    viennamini::numeric get_acceptor_doping(int segment_index);
    viennamini::numeric get_donator_doping(int segment_index);
    
    bool has_permittivity(int segment_index);
    bool has_acceptor_doping(int segment_index);
    bool has_donator_doping(int segment_index);

    void set_recombination        (int segment_index, recombination::recombination_ids id);
    recombination::recombination_ids get_recombination(int segment_index);

    void set_mobility        (int segment_index, mobility::mobility_ids id);
    mobility::mobility_ids get_mobility(int segment_index);

    void update_problem_description();

    std::string& description();

    IndicesType&   segment_indices();
    IndicesType&   contact_segments_indices();
    IndicesType&   oxide_segments_indices();
    IndicesType&   semiconductor_segments_indices();

    viennamaterials::accessor_handle&  matlib_material();
    viennamaterials::accessor_handle&  matlib_model();
    viennamaterials::accessor_handle&  matlib_parameter();
    viennamaterials::accessor_handle&  matlib_data();

    std::ostream & stream();

  private:
 
  
    GenericMeshType               mesh_;

    IndicesType                segment_indices_;
    IndicesType                contact_segments_indices_;
    IndicesType                oxide_segments_indices_;
    IndicesType                semiconductor_segments_indices_;

    IndexMapType               contact_semiconductor_interfaces_;
    IndexMapType               contact_oxide_interfaces_;
    std::string                description_;

    IndexKeysType              segment_names_;
    IndexKeysType              segment_materials_;
    SegmentRolesType           segment_roles_;
    SegmentRecombinationsType  segment_recombinations_;
    SegmentMobilityType        segment_mobility_;
    IndexValuesType            segment_permittivity_;
    IndexValuesType            segment_donator_doping_;
    IndexValuesType            segment_acceptor_doping_;

    viennamini::numeric        temperature_;

    viennamini::material_library_handle  matlib_;

    viennamaterials::accessor_handle matlib_material_;
    viennamaterials::accessor_handle matlib_model_;
    viennamaterials::accessor_handle matlib_parameter_;
    viennamaterials::accessor_handle matlib_data_;

    std::ostream& stream_;
  };


} // viennamini

#endif


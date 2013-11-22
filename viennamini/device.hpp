
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
#include <vector>

#include "viennamini/forwards.h"
#include "viennamini/material_library.hpp"

#include "boost/variant.hpp"


namespace viennamini 
{

  /** @brief Exception for the case that the device type is not supported */
  class device_not_supported_exception : public std::runtime_error {
  public:
    device_not_supported_exception()                        : std::runtime_error("") {}
    device_not_supported_exception(std::string const & str) : std::runtime_error(str) {}
  };


  class device
  {
  private:
    enum segment_role_ids
    {
      unidentified,
      contact, 
      oxide,
      semiconductor
    };
    
    typedef std::map<std::size_t, segment_role_ids>                                          SegmentRolesType;

  
  public:
    // [JW] note that the first type in a boost::variant must be a default constructible object
    typedef boost::variant<null, segmesh_line_1d_ptr,         segmesh_triangular_2d_ptr,         segmesh_tetrahedral_3d_ptr>         GenericMeshType;
    typedef boost::variant<null, problem_description_line_1d, problem_description_triangular_2d, problem_description_tetrahedral_3d> GenericProblemDescriptionType;
    
    typedef std::vector<std::size_t>                                                       IndicesType;  
    typedef std::map<std::size_t, std::size_t>                                             IndexMapType;
    typedef std::map<std::size_t, std::string>                                             IndexKeysType;
    typedef std::map<std::size_t, viennamini::numeric>                                     IndexValuesType;

    typedef GenericMeshType                                                                generic_mesh_type;
    typedef GenericProblemDescriptionType                                                  generic_problem_description_type;
    typedef IndicesType                                                                    indices_type;
    typedef IndexMapType                                                                   index_map_type;
    typedef IndexKeysType                                                                  index_keys_type;

    device();

    void make_line1d();
    void make_triangular2d();
    void make_tetrahedral3d();

    bool is_line1d();
    bool is_triangular2d();
    bool is_tetrahedral3d();

    segmesh_line_1d&  get_segmesh_line_1d();
    segmesh_triangular_2d&  get_segmesh_triangular_2d();
    segmesh_tetrahedral_3d& get_segmesh_tetrahedral_3d();

    problem_description_line_1d&        get_problem_description_line_1d();
    problem_description_triangular_2d&  get_problem_description_triangular_2d();
    problem_description_tetrahedral_3d& get_problem_description_tetrahedral_3d();
    
    void make_contact       (int segment_index);
    void make_oxide         (int segment_index);
    void make_semiconductor (int segment_index);
    
    bool is_contact                 (int segment_index);
    bool is_contact_at_oxide        (int segment_index);
    bool is_contact_at_semiconductor(int segment_index);
    bool is_oxide                   (int segment_index);
    bool is_semiconductor           (int segment_index);

    std::size_t get_adjacent_semiconductor_segment_for_contact(int segment_index);
    std::size_t get_adjacent_oxide_segment_for_contact        (int segment_index);

    void update();

    GenericMeshType               & generic_mesh();
    GenericProblemDescriptionType & generic_problem_description();
    viennamini::material_library  & material_library();


    void read(std::string const& filename, viennamini::line_1d const&);
    void read(std::string const& filename, viennamini::triangular_2d const&);
    void read(std::string const& filename, viennamini::tetrahedral_3d const&);
    
    void read_material_library(std::string const& filename);
    
    void write(std::string const& filename);
  
    void scale(viennamini::numeric factor);
    
    void set_name                 (int segment_index, std::string const& new_name);
    void set_material             (int segment_index, std::string const& new_material);
    std::string get_name          (int segment_index);
    std::string get_material      (int segment_index);


    void set_contact_potential    (int segment_index, viennamini::numeric potential);
    void add_contact_workfunction (int segment_index, viennamini::numeric workfunction);
    void set_relative_permittivity(int segment_index, viennamini::numeric epsr);
    void set_acceptor_doping      (int segment_index, viennamini::numeric NA);
    void set_donator_doping       (int segment_index, viennamini::numeric ND);

    viennamini::numeric get_acceptor_doping(int segment_index);
    viennamini::numeric get_donator_doping(int segment_index);

    void update_problem_description();

    std::string& description();

    IndicesType&   contact_segments_indices();
    IndicesType&   oxide_segments_indices();
    IndicesType&   semiconductor_segments_indices();

  private:
    GenericMeshType               generic_mesh_;
    GenericProblemDescriptionType generic_problem_description_;

    IndicesType                contact_segments_indices_;
    IndicesType                oxide_segments_indices_;
    IndicesType                semiconductor_segments_indices_;

    IndexMapType               contact_semiconductor_interfaces_;
    IndexMapType               contact_oxide_interfaces_;
    std::string                description_;
    
    IndexKeysType              segment_names_;
    IndexKeysType              segment_materials_;
    SegmentRolesType           segment_roles_;
    IndexValuesType            segment_donator_doping_;
    IndexValuesType            segment_acceptor_doping_;
    
    viennamini::material_library_handle  matlib_;
  };


} // viennamini

#endif


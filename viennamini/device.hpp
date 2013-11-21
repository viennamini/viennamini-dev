
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
#include "viennamini/device_segment_parameters.hpp"

#include "boost/variant.hpp"


namespace viennamini 
{

  class device
  {
  public:
    // [JW] note that the first type in a boost::variant must be a default constructible object
    typedef boost::variant<null, segmesh_triangular_2d_ptr, segmesh_tetrahedral_3d_ptr>    GenericMeshType;
    typedef double                                                                         NumericType;
    typedef viennamini::segment_parameters<NumericType>                                    SegmentParametersType;
    typedef std::map<int, SegmentParametersType >                                          ParametersType;
    typedef std::vector<std::size_t>                                                       IndicesType;  
    typedef std::map<std::size_t, std::size_t>                                             IndexMapType;

    typedef GenericMeshType                                                                generic_mesh_type;
    typedef NumericType                                                                    numeric_type;
    typedef SegmentParametersType                                                          segment_parameters_type;
    typedef ParametersType                                                                 parameters_type;
    typedef IndicesType                                                                    indices_type;
    typedef IndexMapType                                                                   index_map_type;

    device();

    void make_triangular2d();
    void make_tetrahedral3d();
    
    bool is_triangular2d();
    bool is_tetrahedral3d();

    segmesh_triangular_2d&  get_segmesh_triangular_2d();
    segmesh_tetrahedral_3d& get_segmesh_tetrahedral_3d();
    
    std::string& name(int id);
    std::string& material(int id);

    void make_contact(int id);
    void make_oxide(int id);
    void make_semiconductor(int id);
    void make_manual(int id);
    
    NumericType& contact_potential(int id);
    NumericType& workfunction(int id);
    NumericType& NA_max(int id);
    NumericType& ND_max(int id);
    NumericType& epsr(int id);

    bool is_contact(int id);
    bool is_contact_at_oxide(int id);
    bool is_contact_at_semiconductor(int id);
    bool is_oxide(int id);
    bool is_semiconductor(int id);
    bool is_manual(int id);

    std::size_t get_adjacent_semiconductor_segment_for_contact(int id);
    std::size_t get_adjacent_oxide_segment_for_contact(int id);

    void update();

    GenericMeshType         & generic_mesh();
    SegmentParametersType   & segment_parameters(int id);

    viennamini::data_storage_handle& storage();

    void read(std::string const& filename, viennamini::triangular_2d const&);
    void read(std::string const& filename, viennamini::tetrahedral_3d const&);
    
    void write(std::string const& filename);
  
    void set_default_parameters();

    void scale(numeric_type factor);
    
    std::string& description();

    IndicesType&   contact_segments_indices();
    IndicesType&   oxide_segments_indices();
    IndicesType&   semiconductor_segments_indices();

  private:
    viennamini::data_storage_handle  storage_;

    GenericMeshType            generic_mesh_;
    ParametersType             parameters_;

    IndicesType                contact_segments_indices_;
    IndicesType                oxide_segments_indices_;
    IndicesType                semiconductor_segments_indices_;

    IndexMapType               contact_semiconductor_interfaces_;
    IndexMapType               contact_oxide_interfaces_;
    std::string                description_;
  };


} // viennamini

#endif


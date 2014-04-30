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
#include "viennamini/value_accessor.hpp"
#include "viennamini/quantity_converter.hpp"
#include "viennamini/utils/enable_if.hpp"

namespace viennamini
{

  template<typename T>
  struct is_accessor
  {
  private:
    typedef char true_type;
    struct false_type{ true_type _[2]; };

    template <typename U>
    static true_type has_result_type(typename U::result_type *);

    template <typename U>
    static false_type has_result_type(...);

  public:
    enum { value = (sizeof(has_result_type<T>(0)) == sizeof(true_type)) };
//    static const bool value = (sizeof(has_result_type<T>(0)) == sizeof(true_type));
  };

  class device_exception : public std::runtime_error {
  public:
    device_exception(std::string const & str) : std::runtime_error(str) {}
  };


  class device
  {
  private:
    typedef std::map<std::size_t, role::segment_role_ids>                                         SegmentRolesType;
    typedef std::map<std::size_t, recombination::recombination_ids>                               SegmentRecombinationsType;
//    typedef std::map<std::size_t, mobility::mobility_ids>                                         SegmentMobilityType;


    typedef generic_mesh    GenericMeshType;

    // TODO check if these types are still required
    typedef std::vector<int>                                                               IndicesType;
    typedef std::map<int, int>                                                             IndexMapType;
    typedef std::map<int, std::string>                                                     IndexKeysType;
    typedef std::map<int, viennamini::numeric>                                             IndexValuesType;
    typedef std::map<std::string, std::map<int, viennamini::sparse_values> >               QuantityDatabaseType;
    typedef std::map<std::string, std::map<int, viennamini::numeric> >                     ContactDatabaseType;

  public:
    typedef GenericMeshType                                                                generic_mesh_type;
    typedef IndicesType                                                                    indices_type;
    typedef IndexMapType                                                                   index_map_type;
    typedef IndexKeysType                                                                  index_keys_type;
    typedef QuantityDatabaseType                                                           quantity_database_type;
    typedef ContactDatabaseType                                                            contact_database_type;

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

    void make(viennamini::role::segment_role_ids role, int segment_index, std::string const& name, std::string const& material);

    bool is_contact                 (int segment_index);
    bool is_contact_at_oxide        (int segment_index);
    bool is_contact_at_semiconductor(int segment_index);
    bool is_oxide                   (int segment_index);
    bool is_semiconductor           (int segment_index);

    viennamini::role::segment_role_ids get_segment_role(int segment_index);
    std::string                        get_segment_role_string(int segment_index);


    void set_temperature(viennamini::numeric const& temp_val);

    int get_adjacent_semiconductor_segment_for_contact(int segment_index);
    int get_adjacent_oxide_segment_for_contact        (int segment_index);
    int get_adjacent_segment_for_contact              (int segment_index);

    void update();

    GenericMeshType                 & mesh();
    viennamaterials::library_handle & material_library();


    void read(std::string const& filename, viennamini::line_1d const&);
    void read(std::string const& filename, viennamini::triangular_2d const&);
    void read(std::string const& filename, viennamini::tetrahedral_3d const&);

    void read_material_database(std::string const& filename);
    void set_material_database(viennamaterials::library_handle& matlib);

    void read_unit_database(std::string const& filename);

    void write(std::string const& filename);

    void scale(viennamini::numeric factor);

    void set_name     (int segment_index, std::string const& new_name);
    void set_material (int segment_index, std::string const& new_material);
    void set_role     (int segment_index, viennamini::role::segment_role_ids const& new_role);

    std::string&                        get_name      (int segment_index);
    std::string&                        get_material  (int segment_index);
    viennamini::role::segment_role_ids& get_role      (int segment_index);

    /// Distribute the value of a single quantity on the entire device
    void set_quantity (std::string          const& quantity_name,
                       viennamini::numeric         value,
                       std::string          const& unit);

    /// Distribute the value of a single quantity on a specific device segment
    void set_quantity (std::string          const& quantity_name,
                       int                         segment_index,
                       viennamini::numeric         value,
                       std::string          const& unit);

    /// Distribute the values of a single quantity on the entire device
    void set_quantity (std::string          const& quantity_name,
                       viennamini::sparse_values & values,
                       std::string          const& unit);

    /// Distribute the values of a single quantity on a specific device segment
    void set_quantity (std::string          const& quantity_name,
                       int                         segment_index,
                       viennamini::sparse_values & values,
                       std::string          const& unit);

    /// Distribute the values of a single quantity on the entire device
    void set_quantity (std::string          const& quantity_name,
                       viennamini::dense_values  & values,
                       std::string          const& unit);

    /// Distribute the values via an accessor of a single quantity on a specific device segment
    template<typename AccessorT>
    typename viennamini::enable_if< viennamini::is_accessor<AccessorT>::value >::type
    set_quantity (std::string                                      const& quantity_name,
                  int                                                     segment_index,
                  AccessorT                                               accessor)
    {
      if(this->is_line1d())
      {
        typedef viennagrid::result_of::cell_range<segment_line_1d>::type      CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type     CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_line_1d>::type               CellType;

        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_line_1d().segmentation[segment_index]);
        try{
          for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
          {
            std::size_t cell_index = (*cit).id().get();
            quantity_database_[quantity_name][segment_index][cell_index] = accessor(cell_index);
          }
        }
        catch(std::exception const& e)
        {
          throw device_exception("Error with distributing sparse values of quantity \""+quantity_name+"\". Verify cell-value mapping!");
        }
      }
      else
      if(this->is_triangular2d())
      {
        typedef viennagrid::result_of::cell_range<segment_triangular_2d>::type  CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type       CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_triangular_2d>::type           CellType;

        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_triangular_2d().segmentation[segment_index]);
        try{
          for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
          {
            std::size_t cell_index = (*cit).id().get();
            quantity_database_[quantity_name][segment_index][cell_index] = accessor(cell_index);
          }
        }
        catch(std::exception const& e)
        {
          throw device_exception("Error with distributing sparse values of quantity \""+quantity_name+"\". Verify cell-value mapping!");
        }
      }
      else
      if(this->is_tetrahedral3d())
      {
        typedef viennagrid::result_of::cell_range<segment_tetrahedral_3d>::type   CellOnSegmentRange;
        typedef viennagrid::result_of::iterator<CellOnSegmentRange>::type         CellOnSegmentIterator;
        typedef viennagrid::result_of::cell<mesh_tetrahedral_3d>::type            CellType;

        CellOnSegmentRange cells = viennagrid::elements<CellType>(get_segmesh_tetrahedral_3d().segmentation[segment_index]);
        try{
          for(CellOnSegmentIterator cit = cells.begin(); cit != cells.end(); cit++)
          {
            std::size_t cell_index = (*cit).id().get();
            quantity_database_[quantity_name][segment_index][cell_index] = accessor(cell_index);
          }
        }
        catch(std::exception const& e)
        {
          throw device_exception("Error with distributing sparse values of quantity \""+quantity_name+"\". Verify cell-value mapping!");
        }
      }
    }

public:
    ///
    viennamini::value_accessor get_quantity_value_accessor (std::string const& quantity_name, int segment_index);

    ///
    viennamini::numeric get_quantity_value (std::string const& quantity_name, int segment_index, std::size_t cell_index);

    /// Test whether a quantity is stored for each cell of a specific segment
    bool                  has_quantity (std::string const& quantity_name, int segment_index);

    /// Store a contact value
    void                  set_contact_quantity (std::string const& quantity_name, int segment_index, viennamini::numeric  value, std::string const& unit);

    /// Retrieve a contact value
    viennamini::numeric   get_contact_quantity_value (std::string const& quantity_name, int segment_index);

    /// Test whether a contact value is stored for a specific segment
    bool                  has_contact_quantity (std::string const& quantity_name, int segment_index);

    void set_recombination        (int segment_index, recombination::recombination_ids id);
    recombination::recombination_ids get_recombination(int segment_index);

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

    QuantityDatabaseType      quantity_database_;
    ContactDatabaseType       contact_database_;

    viennamaterials::library_handle  matlib_;

    viennamaterials::accessor_handle matlib_material_;
    viennamaterials::accessor_handle matlib_model_;
    viennamaterials::accessor_handle matlib_parameter_;
    viennamaterials::accessor_handle matlib_data_;

    std::ostream& stream_;
    viennamini::quantity_converter_handle converter_;
  };


} // viennamini

#endif

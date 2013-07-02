#ifndef VIENNAMINI_DEVICE_HPP
#define VIENNAMINI_DEVICE_HPP


// ViennaGrid includes:
#include "viennagrid/domain.hpp"
#include "viennagrid/config/simplex.hpp"

namespace viennamini {

template<typename DomainT>
struct device
{
  typedef double                              Numeric;
  typedef std::map<std::size_t, std::string>  IndexKeys;
  typedef std::vector<std::size_t>            Indices;
  typedef std::map<std::size_t, Numeric>      IndexValues;
  typedef DomainT                             Domain;


  device(DomainT& domain) : domain(domain) {}

  /** 
      @brief Stores a name string for a given segment index
  */
  void assign_name(std::size_t segment_index, std::string const& name)
  {
    segment_names[segment_index] = name;
  }

  /** 
      @brief Stores a material ID string for a given segment index
  */
  void assign_material(std::size_t segment_index, std::string const& material_id)
  {
    segment_materials[segment_index] = material_id;
  }

  /** 
      @brief Identifies the segment to be a contact
  */
  void assign_contact(std::size_t segment_index)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_contact(): segment " << segment_index << std::endl;
  #endif
    contact_segments.push_back(segment_index);
  }

  /** 
      @brief Identifies the segment to be a oxide
  */
  void assign_oxide(std::size_t segment_index)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_oxide(): segment " << segment_index << std::endl;
  #endif
    oxide_segments.push_back(segment_index);
  }

  /** 
      @brief Identifies the segment to be a semiconductor
  */
  template<typename NumericT>
  void assign_semiconductor(std::size_t segment_index, NumericT const& ND, NumericT const& NA)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_semiconductor(): segment " << segment_index << std::endl;
  #endif
    semiconductor_segments.push_back(segment_index);

    segment_donators[segment_index] = ND;
    segment_acceptors[segment_index] = NA;
  }

  IndexKeys& get_segment_names()          { return segment_names; }
  IndexKeys& get_segment_materials()      { return segment_materials; }
  Indices&   get_contact_segments()       { return contact_segments; }
  Indices&   get_oxide_segments()         { return oxide_segments; }
  Indices&   get_semiconductor_segments() { return semiconductor_segments; }

  Numeric get_donator(std::size_t segment_index) { return segment_donators[segment_index]; }
  Numeric get_acceptor(std::size_t segment_index) { return segment_acceptors[segment_index]; }


  DomainT& get_domain() { return domain; }

  // -----
private:
  DomainT& domain;

  IndexKeys               segment_names;
  IndexKeys               segment_materials;
  Indices                 contact_segments;
  Indices                 oxide_segments;
  Indices                 semiconductor_segments;
  IndexValues             segment_donators;
  IndexValues             segment_acceptors;
};

} // viennamini

#endif


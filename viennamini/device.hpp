#ifndef VIENNAMINI_DEVICE_HPP
#define VIENNAMINI_DEVICE_HPP


// ViennaGrid includes:
#include "viennagrid/domain.hpp"
#include "viennagrid/config/simplex.hpp"

namespace viennamini {

template<typename DomainT, typename MatlibT>
struct Device
{
  typedef double                              Numeric;
  typedef std::map<std::size_t, std::string>  IndexKeys;
  typedef std::vector<std::size_t>            Indices;
  typedef std::map<std::size_t, Numeric>      IndexValues;


  Device(DomainT& domain, MatlibT& matlib) : domain(domain), matlib(matlib) {}


  void assign_name(std::size_t segment_index, std::string const& name)
  {
    segment_names[segment_index] = name;
  }

  void assign_material(std::size_t segment_index, std::string const& material_id)
  {
    segment_materials[segment_index] = material_id;
  }

  void assign_contact(std::size_t segment_index)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_contact(): segment " << segment_index << std::endl;
  #endif
    contact_segments.push_back(segment_index);

    viennafvm::set_quantity_region(eps_key,     domain.segments()[segment_index], false);
    viennafvm::set_quantity_region(builtin_key, domain.segments()[segment_index], false);
  }

  void assign_oxide(std::size_t segment_index)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_oxide(): segment " << segment_index << std::endl;
  #endif
    //std::cout << "  epsr: " << matlib.getParameterValue(segment_materials[segment_index], "permittivity") << std::endl;


    oxide_segments.push_back(segment_index);

    viennafvm::set_quantity_region(eps_key, domain.segments()[segment_index], true);
    viennafvm::set_quantity_value (eps_key, domain.segments()[segment_index],
                                   matlib.getParameterValue(
                                     segment_materials[segment_index], "permittivity") * viennamini::eps0::val()); // TODO

    viennafvm::set_quantity_region(builtin_key, domain.segments()[segment_index], false);
  }

  template<typename NumericT>
  void assign_semiconductor(std::size_t segment_index, NumericT const& ND, NumericT const& NA)
  {
  #ifdef VIENNAMINI_DEBUG
    std::cout << "* assign_semiconductor(): segment " << segment_index << std::endl;
  #endif
//    std::cout << "  ND: " << ND << " NA: " << NA << std::endl;
//    std::cout << "  epsr: " << matlib.getParameterValue(segment_materials[segment_index], "permittivity") << std::endl;

    semiconductor_segments.push_back(segment_index);

    segment_donators[segment_index] = ND;
    segment_acceptors[segment_index] = NA;

    viennafvm::set_quantity_region(eps_key, domain.segments()[segment_index], true);
    viennafvm::set_quantity_value (eps_key, domain.segments()[segment_index],
                                   matlib.getParameterValue(
                                     segment_materials[segment_index], "permittivity") * viennamini::eps0::val()); // TODO

    viennafvm::set_quantity_region(ND_key, domain.segments()[segment_index], true);
    viennafvm::set_quantity_value (ND_key, domain.segments()[segment_index], ND);

    viennafvm::set_quantity_region(NA_key, domain.segments()[segment_index], true);
    viennafvm::set_quantity_value (NA_key, domain.segments()[segment_index], NA);
  }

  IndexKeys& get_segment_names()          { return segment_names; }
  IndexKeys& get_segment_materials()      { return segment_materials; }
  Indices&   get_contact_segments()       { return contact_segments; }
  Indices&   get_oxide_segments()         { return oxide_segments; }
  Indices&   get_semiconductor_segments() { return semiconductor_segments; }

  Numeric get_donator(std::size_t segment_index) { return segment_donators[segment_index]; }
  Numeric get_acceptor(std::size_t segment_index) { return segment_acceptors[segment_index]; }


  DomainT& get_domain() { return domain; }
  MatlibT& get_matlib() { return matlib; }

  // -----
private:
  DomainT& domain;
  MatlibT& matlib;

  IndexKeys               segment_names;
  IndexKeys               segment_materials;
  Indices                 contact_segments;
  Indices                 oxide_segments;
  Indices                 semiconductor_segments;
  IndexValues             segment_donators;
  IndexValues             segment_acceptors;

  viennamini::permittivity_key       eps_key;
  viennamini::donator_doping_key     ND_key;
  viennamini::acceptor_doping_key    NA_key;
  viennamini::builtin_potential_key  builtin_key;

};






//template <typename ConfigType>
//void assign(viennamini::contact_key           const& contact,
//            viennagrid::segment_t<ConfigType> const& seg,
//            std::string                       const& material_string)
//{
//    viennamini::permittivity_key       eps;
//    viennamini::builtin_potential_key  builtin;
//    viennamini::material_key           material;

//    viennafvm::set_quantity_region(eps,     domain.segments()[si], false);
//    viennafvm::set_quantity_region(builtin, domain.segments()[si], false);
    
//    viennadata::access<viennamini::material_key, std::string >(material)(seg) = material_string;
//}

//template <typename ConfigType>
//void assign(viennamini::oxide_key             const& oxide,
//            viennagrid::segment_t<ConfigType> const& seg,
//            std::string                       const& material_string)
//{
//    viennamini::permittivity_key       eps;
//    viennamini::builtin_potential_key  builtin;
//    viennamini::material_key           material;

//    viennafvm::set_quantity_region(eps, seg, true);
//    viennafvm::set_quantity_value (eps, seg,
//                                   matlib.getParameterValue(
//                                       siter->material.toStdString(), "permittivity") * eps0);

//    viennafvm::set_quantity_region(builtin_key, domain.segments()[si], false); // TODO not for oxides?!

//    viennafvm::disable_quantity(domain.segments()[si], my_simulator.quantity_electron_density());
//    viennafvm::disable_quantity(domain.segments()[si], my_simulator.quantity_hole_density());
//}

} // viennamini

#endif


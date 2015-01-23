/* =======================================================================
   Copyright (c) 2011-2015, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */



// ViennaMini includes
//
#include "viennamini/configure_simulator_xml.hpp"
#include "viennamini/utils/string.hpp"
#include "viennamini/utils/convert.hpp"
#include "viennamini/utils/environment.hpp"
#include "external/pugixml/pugixml.hpp"

namespace viennamini {

namespace detail {

template<typename TargetT>
struct pugixml_query_impl
{
  static TargetT eval(pugi::xml_document const& doc, std::string const& native_query)
  {
    return viennamini::convert<TargetT>( pugi::xpath_query(native_query.c_str()).evaluate_string(doc) );
  }
};

template<>
struct pugixml_query_impl <std::string>
{
  static std::string eval(pugi::xml_document const& doc, std::string const& native_query)
  {
    return pugi::xpath_query(native_query.c_str()).evaluate_string(doc);
  }
};

} // detail


template<typename TargetT>
inline TargetT pugixml_query(pugi::xml_document const& doc, std::string const& native_query)
{
  return detail::pugixml_query_impl<TargetT>::eval(doc, native_query);
}

inline bool pugixml_has_node(pugi::xml_document const& doc, std::string const& native_query)
{
  return doc.select_node(native_query.c_str());
}

inline bool pugixml_has_attribute(pugi::xml_document const& doc, std::string const& native_query, std::string const& attribute)
{
  return !doc.select_node(native_query.c_str()).node().attribute(attribute.c_str()).empty();
}

void configure_simulator_xml(viennamini::simulator& sim, std::string const& configuration_file)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(configuration_file.c_str());
  if(!result)
  {
    throw configure_simulator_xml_exception("Configure Simulator XML encountered errors during parsing, attr value: [" +
      std::string(doc.child("node").attribute("attr").value()) + "]\n" +
      "Description: " + std::string(result.description()) + "\n" +
      "Error offset: " + viennamini::convert<std::string>(result.offset) );// + " (error at [..." + viennamini::convert<std::string>(source + result.offset) + "]\n\n");
  }
  // extract the meshfile
  //
  std::string meshfile_prefix_path = viennamini::extract_environment_variable(pugixml_query<std::string>(doc, "/simulation/mesh/@env_prefix"));
  if(!meshfile_prefix_path.empty()) meshfile_prefix_path += "/";
  std::string meshfile = meshfile_prefix_path + pugixml_query<std::string>(doc, "/simulation/mesh/@file");
  std::string meshtype = pugixml_query<std::string>(doc, "/simulation/mesh/@type");
  // TODO dirty, temporary solution
  // this dispatch should become obsolete with the new runtime domain
  if(meshtype == "triangular_2d")
    sim.device().read(meshfile, viennamini::triangular_2d());
  else
  if(meshtype == "tetrahedral_3d")
    sim.device().read(meshfile, viennamini::tetrahedral_3d());
  else throw configure_simulator_xml_exception("Meshtype not supported!");

  // extract the material database file
  //
  std::string material_prefix_path = viennamini::extract_environment_variable(pugixml_query<std::string>(doc, "/simulation/material_db/@env_prefix"));
  if(!material_prefix_path.empty()) material_prefix_path += "/";
  std::string materialfile = material_prefix_path + pugixml_query<std::string>(doc, "/simulation/material_db/@file");
  sim.device().read_material_database(materialfile);

  // extract the unit database file
  //
  std::string unit_prefix_path = viennamini::extract_environment_variable(pugixml_query<std::string>(doc, "/simulation/unit_db/@env_prefix"));
  if(!unit_prefix_path.empty()) unit_prefix_path += "/";
  std::string unitfile = unit_prefix_path + pugixml_query<std::string>(doc, "/simulation/unit_db/@file");
  sim.device().read_unit_database(unitfile);

  // process mesh scaling
  //
  if(pugixml_has_attribute(doc, "/simulation/device/scale", "value"))
    sim.device().scale(pugixml_query<double>(doc, "/simulation/device/scale/@value"));

  // process the segment roles
  //
  pugi::xpath_node_set roles = doc.select_nodes("/simulation/device/role");
  for (pugi::xpath_node_set::const_iterator role_iter = roles.begin(); role_iter != roles.end(); ++role_iter)
  {
    sim.device().make(
      viennamini::role::key_to_id(viennamini::convert<std::string>(role_iter->node().attribute("type").value())),
      viennamini::convert<int>(role_iter->node().attribute("segment").value()),
      viennamini::convert<std::string>(role_iter->node().attribute("name").value()),
      viennamini::convert<std::string>(role_iter->node().attribute("material").value())
    );
  }

  // process the quantities
  //
  pugi::xpath_node_set quantities = doc.select_nodes("/simulation/device/quantity");
  for (pugi::xpath_node_set::const_iterator quantity_iter = quantities.begin(); quantity_iter != quantities.end(); ++quantity_iter)
  {
    // process segment-specific quantity by checking whether the
    // quantity entry offers a segment id
    if(!quantity_iter->node().attribute("segment").empty())
    {
      sim.device().set_quantity(
        viennamini::convert<std::string>(quantity_iter->node().attribute("name").value()),
        viennamini::convert<int>(quantity_iter->node().attribute("segment").value()),
        viennamini::convert<double>(quantity_iter->node().attribute("value").value()),
        viennamini::convert<std::string>(quantity_iter->node().attribute("unit").value())
      );
    }
    // if not, this is a domain-wide quantity
    else
    {
      sim.device().set_quantity(
        viennamini::convert<std::string>(quantity_iter->node().attribute("name").value()),
        viennamini::convert<double>(quantity_iter->node().attribute("value").value()),
        viennamini::convert<std::string>(quantity_iter->node().attribute("unit").value())
      );
    }
  }

  // process the linear solver parameters
  //
  if(pugixml_has_node(doc, "/simulation/solver/linear/break_tolerance"))
    sim.config().linear_breaktol() = pugixml_query<double>(doc, "/simulation/solver/linear/break_tolerance/@value");
  if(pugixml_has_node(doc, "/simulation/solver/linear/max_iterations"))
    sim.config().linear_iterations() = pugixml_query<long>(doc, "/simulation/solver/linear/max_iterations/@value");

  // process the nonlinear solver parameters
  //
  if(pugixml_has_node(doc, "/simulation/solver/nonlinear/break_tolerance"))
    sim.config().nonlinear_breaktol() = pugixml_query<double>(doc, "/simulation/solver/nonlinear/break_tolerance/@value");
  if(pugixml_has_node(doc, "/simulation/solver/nonlinear/max_iterations"))
    sim.config().nonlinear_iterations() = pugixml_query<long>(doc, "/simulation/solver/nonlinear/max_iterations/@value");
  if(pugixml_has_node(doc, "/simulation/solver/nonlinear/damping"))
    sim.config().damping() = pugixml_query<double>(doc, "/simulation/solver/nonlinear/damping/@value");

  // set the transport model
  //
  sim.config().model().set_pdeset(viennamini::pdeset::key_to_id(pugixml_query<std::string>(doc, "/simulation/model/pde_set/@type")));

  // set the discretization
  //
  sim.config().model().set_discretization(viennamini::discret::key_to_id(pugixml_query<std::string>(doc, "/simulation/model/discretization/@type")));

  // process the contacts
  //
  pugi::xpath_node_set contacts = doc.select_nodes("/simulation/contacts/contact");
  for (pugi::xpath_node_set::const_iterator contact_iter = contacts.begin(); contact_iter != contacts.end(); ++contact_iter)
  {
    sim.device().set_contact_quantity(
      viennamini::convert<std::string>(contact_iter->node().attribute("type").value()),
      viennamini::convert<int>(contact_iter->node().attribute("segment").value()),
      viennamini::convert<double>(contact_iter->node().attribute("value_single").value()),
      viennamini::convert<std::string>(contact_iter->node().attribute("unit").value()));
  }



}

} // viennamini

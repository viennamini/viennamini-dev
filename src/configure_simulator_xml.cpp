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

inline std::string pugixml_indent_string()
{
  return std::string("  ");
}

inline std::string pugixml_query(pugi::xml_document const& doc, std::string const& native_query)
{
  return pugi::xpath_query(native_query.c_str()).evaluate_string(doc);
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
  std::string meshfile = viennamini::extract_environment_variable(pugixml_query(doc, "/simulation/mesh/@env_prefix")) + pugixml_query(doc, "/simulation/mesh/@file");
  std::cout << "meshfile: " << meshfile << std::endl;

//  std::cout << pugixml_query(doc, "/simulation/device/scale/@value") << std::endl;



}



} // viennamini

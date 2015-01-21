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

#include "viennamini/configure_simulator.hpp"
#include "viennamini/utils/file_extension.hpp"

#include "external/pugixml/pugixml.hpp"

namespace viennamini {

void configure_simulator_xml(viennamini::simulator& sim, std::string const& configuration_file)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(configuration_file.c_str());
  if(!result)
  {
    throw configure_simulator_exception("Configure Simulator XML encountered errors during parsing, attr value: [" +
      std::string(doc.child("node").attribute("attr").value()) + "]\n" +
      "Description: " + std::string(result.description()) + "\n");
  }


}

void configure_simulator(viennamini::simulator& sim, std::string const& configuration_file)
{
  if(!viennamini::file_exists(configuration_file))
    throw configure_simulator_exception("Configuration file does not exist!");

  std::string configuration_file_ending = viennamini::file_extension(configuration_file);
  if(configuration_file_ending == "xml")
  {
    configure_simulator_xml(sim, configuration_file);
  }
  else throw configure_simulator_exception("Configuration file type not supported!");
}

} // viennamini

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
#include "viennamini/configure_simulator.hpp"
#include "viennamini/configure_simulator_xml.hpp"
#include "viennamini/utils/file_extension.hpp"

namespace viennamini {

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

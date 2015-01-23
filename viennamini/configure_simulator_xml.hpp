#ifndef VIENNAMINI_CONFIGURE_SIMULATOR_XML_HPP
#define VIENNAMINI_CONFIGURE_SIMULATOR_XML_HPP

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
#include "viennamini/simulator.hpp"

namespace viennamini {

/**
 * @brief Exception class for the XML-based simulation configurator
 */
class configure_simulator_xml_exception : public std::runtime_error {
public:
  configure_simulator_xml_exception(std::string const & str) : std::runtime_error(str) {}
};

/**
 * @brief Configures a ViennaMini simulator object according to an input XML-based configuration file
 * @param sim The ViennaMini simulator object
 * @param configuration_file Path to the input XML configuration file
 */
void configure_simulator_xml(viennamini::simulator& sim, std::string const& configuration_file);



} // viennamini

#endif

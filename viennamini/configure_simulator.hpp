#ifndef VIENNAMINI_CONFIGURE_SIMULATOR_HPP
#define VIENNAMINI_CONFIGURE_SIMULATOR_HPP

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

class configure_simulator_exception : public std::runtime_error {
public:
  configure_simulator_exception(std::string const & str) : std::runtime_error(str) {}
};

void configure_simulator_xml(viennamini::simulator& sim, std::string const& configuration_file);
void configure_simulator    (viennamini::simulator& sim, std::string const& configuration_file);


} // viennamini

#endif

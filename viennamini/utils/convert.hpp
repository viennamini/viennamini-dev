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

#ifndef VIENNAMINI_UTILS_CONVERT_HPP
#define VIENNAMINI_UTILS_CONVERT_HPP

#include <sstream>

namespace viennamini {

template<typename TargetT, typename SourceT>
TargetT convert(SourceT const& source)
{
  TargetT target;
  std::stringstream sstr;
  sstr << source;
  sstr >> target;
  return target;
}

} // end namespace viennamini

#endif





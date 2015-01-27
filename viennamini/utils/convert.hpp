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

namespace detail {

template<typename TargetT, typename SourceT>
struct convert_impl
{
  static TargetT eval(SourceT const& source)
  {
    TargetT target;
    std::stringstream sstr;
    sstr << source;
    sstr >> target;
    return target;
  }
};

// This specialization is important, as in cases
// where the const char* contains a whitespace, the default
// conversion method would 'cut' the result at the whitespace
//
template<>
struct convert_impl <std::string, const char*>
{
  static std::string eval(const char* source)
  {
    return std::string(source);
  }
};

} // detail

template<typename TargetT, typename SourceT>
inline TargetT convert(SourceT const& source)
{
  return detail::convert_impl<TargetT, SourceT>::eval(source);
}




} // end namespace viennamini

#endif

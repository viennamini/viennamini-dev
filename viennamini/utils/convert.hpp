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
#include <string>

#include <boost/type_traits/is_same.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/lexical_cast.hpp>

namespace viennamini {

template<typename Target>
struct convert
{
  typedef Target result_type; 

  template<typename Source>
  result_type operator()(Source const& source, typename boost::enable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    return source;
  }

  template<typename Source>
  result_type operator()(Source const& source, typename boost::disable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    return boost::lexical_cast<result_type>(source);
  }
};

template<>
struct convert<float>
{
  typedef float result_type; 
   
  template<typename Source>
  result_type operator()(Source const& source, typename boost::enable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    return source;
  }

  template<typename Source>
  result_type operator()(Source const& source, typename boost::disable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    sstr << source;
    sstr >> target;
    return target;
  }
private:
  result_type target;
  std::stringstream sstr;
};

template<>
struct convert<double>
{
  typedef double result_type; 
   
  template<typename Source>
  result_type operator()(Source const& source, typename boost::enable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    return source;
  }

  template<typename Source>
  result_type operator()(Source const& source, typename boost::disable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    sstr << source;
    sstr >> target;
    return target;
  }
private:
  result_type target;
  std::stringstream sstr;
};


template<>
struct convert<std::string>
{
  typedef std::string result_type;
   
  template<typename Source>
  result_type operator()(Source const& source, typename boost::enable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    return source;
  }

  template<typename Source>
  result_type operator()(Source const& source, typename boost::disable_if<boost::is_same<Source,result_type> >::type* dummy = 0)
  {
    sstr << source;
    sstr >> target;
    return target;
  }
private:
  result_type target;
  std::stringstream sstr;
};

} // end namespace viennamini

#endif





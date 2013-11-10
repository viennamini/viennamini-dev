#ifndef VIENNAMINI_DEVICETEMPLATE_HPP
#define VIENNAMINI_DEVICETEMPLATE_HPP

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

#include <vector>
#include <map>

#include "viennagrid/forwards.hpp"
#include "viennagrid/point.hpp"
#include "viennagrid/mesh/element_creation.hpp"

namespace viennamini 
{

class device_template
{
private:
  typedef double                                                                NumericType;
  typedef viennagrid::spatial_point<NumericType, viennagrid::cartesian_cs<3> >  PointType;
  typedef std::map<std::string, PointType>                                      PropertiesType;
public:
  typedef NumericType                   numeric_type;
  typedef PropertiesType                properties_type;
  typedef PointType                     point_type;

  properties_type& properties()  { return properties_; }
  std::string&     description() { return description_; }

  virtual void     generate() = 0;

  PropertiesType properties_;
  std::string    description_;
};

} // viennamini

#endif


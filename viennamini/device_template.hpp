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

#include "viennamini/forwards.h"
#include "viennamini/device.hpp"

#include "viennagrid/forwards.hpp"
#include "viennagrid/point.hpp"
#include "viennagrid/mesh/element_creation.hpp"



namespace viennamini 
{

class device_template
{
private:
  typedef viennamini::numeric                                                   NumericType;
  typedef viennagrid::spatial_point<NumericType, viennagrid::cartesian_cs<3> >  PointType;
  typedef std::map<std::string, PointType>                                      GeometryPropertiesType;

  
public:
  typedef NumericType                   numeric_type;
  typedef GeometryPropertiesType        geometry_properties_type;
  typedef PointType                     point_type;

  device_template(viennamini::data_storage& storage) : storage_(storage), device_(storage) {}
  
  geometry_properties_type& geometry_properties()   { return geometry_properties_;  }
  viennamini::data_storage& storage()               { return storage_;     }
  viennamini::device&       device()                { return device_;      }
  
  void set_geometry_property(std::string const& key, numeric_type x, numeric_type y = 0, numeric_type z = 0)
  {
    geometry_properties_[key] = point_type(x, y, z);
  }

  virtual void        generate()         = 0;
  virtual std::string description()      = 0;

private:
  viennamini::data_storage&  storage_;
  geometry_properties_type   geometry_properties_;
  viennamini::device         device_;
};

} // viennamini

#endif


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
#include "viennamini/config.hpp"

#include "viennagrid/forwards.hpp"
#include "viennagrid/point.hpp"
#include "viennagrid/mesh/element_creation.hpp"

#include "viennamesh/utils/logger.hpp"

#include "boost/shared_ptr.hpp"

namespace viennamini 
{

class device_template
{
protected:
  typedef viennamini::numeric                                                   NumericType;
  typedef viennagrid::spatial_point<NumericType, viennagrid::cartesian_cs<3> >  PointType;
  typedef std::map<std::string, PointType>                                      GeometryPropertiesType;
  
public:
  typedef NumericType                   numeric_type;
  typedef GeometryPropertiesType        geometry_properties_type;
  typedef PointType                     point_type;

  device_template(std::string const& material_library_file, std::ostream& stream = std::cout) 
    : material_library_file_(material_library_file), stream_(stream)
  {
    // deactivate ViennaMesh debug output
//    viennamesh::logger().set_log_level<viennamesh::info_tag>(0);
//    viennamesh::logger().set_log_level<viennamesh::stack_tag>(0);
  }

  virtual ~device_template() {}
  
  geometry_properties_type&        geometry_properties()   { return geometry_properties_;  }
  viennamini::device_handle &      device()                { return device_;      }
  viennamini::config_handle &      config()                { return config_;      }

  void set_geometry_property(std::string const& key, numeric_type x, numeric_type y = 0, numeric_type z = 0)
  {
    geometry_properties_[key] = point_type(x, y, z);
  }

  virtual void         generate()         = 0;

  std::ostream& stream()
  {
    return stream_;
  }

protected:
  geometry_properties_type        geometry_properties_;
  viennamini::device_handle       device_;
  viennamini::config_handle       config_;
  std::string                     material_library_file_;
  std::ostream&                   stream_;
};

} // viennamini

#endif


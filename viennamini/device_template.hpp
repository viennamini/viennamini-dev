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
#include "viennamini/configuration.hpp"

#include "viennagrid/forwards.hpp"
#include "viennagrid/point.hpp"
#include "viennagrid/mesh/element_creation.hpp"

#include "viennamesh/utils/logger.hpp"

namespace viennamini
{

class device_template
{
protected:
  typedef viennamini::numeric                                                   NumericType;
  typedef viennagrid::spatial_point<NumericType, viennagrid::cartesian_cs<3> >  PointType;
  typedef std::map<std::string, PointType>                                      GeometryPropertiesType;
  typedef std::map<std::string, std::size_t>                                    SegmentIndexMapType;

public:
  typedef NumericType                   numeric_type;
  typedef GeometryPropertiesType        geometry_properties_type;
  typedef PointType                     point_type;
  typedef SegmentIndexMapType           segment_index_map_type;

  device_template(std::ostream& stream = std::cout)
    : stream_(stream)
  {
    // deactivate ViennaMesh debug output
    viennamesh::logger().set_log_level<viennamesh::info_tag>(0);
    viennamesh::logger().set_log_level<viennamesh::stack_tag>(0);
  }

  virtual ~device_template() {}

  geometry_properties_type&        geometry_properties()   { return geometry_properties_;  }
  viennamini::device_handle &      device_handle()         { return device_handle_;      }
  viennamini::configuration_handle &      config_handle()         { return config_handle_;      }

  void set_geometry_property(std::string const& key, numeric_type x, numeric_type y = 0, numeric_type z = 0)
  {
    geometry_properties_[key] = point_type(x, y, z);
  }

  virtual void         generate()         = 0;

  std::string&  problem_id()
  {
    return problem_id_;
  }

  std::ostream& stream()
  {
    return stream_;
  }

  segment_index_map_type& segment_indices() { return segment_indices_; }

protected:
  geometry_properties_type        geometry_properties_;
  viennamini::device_handle       device_handle_;
  viennamini::configuration_handle       config_handle_;
  segment_index_map_type          segment_indices_;
  std::string                     problem_id_;
  std::ostream&                   stream_;
};

} // viennamini

#endif


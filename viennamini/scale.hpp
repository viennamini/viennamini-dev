#ifndef VIENNAMINI_SCALE_HPP
#define VIENNAMINI_SCALE_HPP

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

#include "viennamini/fwd.h"

namespace viennamini {

namespace detail {

template<typename MeshT>
void scale_impl(MeshT& mesh, double factor)
{
  viennagrid::scale(mesh, factor);
}

} // detail


void scale(viennamini::device& device, double factor)
{
  if(device.is_triangular2d())
  {
    viennamini::MeshTriangular2DType  & mesh = boost::get<viennamini::MeshTriangular2DType>(device.generic_mesh());
    viennamini::detail::scale_impl(mesh, factor);
  }
  else if(device.is_tetrahedral3d())
  {
    viennamini::MeshTetrahedral3DType & mesh = boost::get<viennamini::MeshTetrahedral3DType>(device.generic_mesh());
    viennamini::detail::scale_impl(mesh, factor);
  }
}

} // viennamini

#endif


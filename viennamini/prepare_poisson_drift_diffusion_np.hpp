#ifndef VIENNAMINI_PREPAREPOISSONDRIFTDIFFUSIONNP_HPP
#define VIENNAMINI_PREPAREPOISSONDRIFTDIFFUSIONNP_HPP

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

template<typename SegmentedMeshT>
void prepare_poisson_drift_diffusion_np_impl(SegmentedMeshT& segmesh)
{

//  //
//  // CONTACTS
//  //
//  viennamini::device::indices_type & contacts = device_.contact_segments_indices();
//  for(typename viennamini::device::indices_type::iterator iter = contacts.begin();
//      iter != contacts.end(); iter++)
//  {
//    typename SegmentedMeshT::segmentation_type::segment_handle_type current_segment = segmesh.segmentation[*cs_it];
//  }

}

} // detail


void prepare_poisson_drift_diffusion_np(viennamini::device& device)
{
  viennamini::device::indices_type & contacts       = device.contact_segments_indices();
  viennamini::device::indices_type & oxides         = device.oxide_segments_indices();
  viennamini::device::indices_type & semiconductors = device.semiconductor_segments_indices();

  if(device.is_triangular2d())
    viennamini::detail::prepare_poisson_drift_diffusion_np_impl(device.get_segmesh_triangular_2d());
  else 
  if(device.is_tetrahedral3d())
    viennamini::detail::prepare_poisson_drift_diffusion_np_impl(device.get_segmesh_tetrahedral_3d());
  else
    std::cout << "detect_interfaces: segmented mesh type not supported" << std::endl;
}

} // viennamini

#endif


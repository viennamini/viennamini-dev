#ifndef VIENNAMINI_DETECTINTERFACES_HPP
#define VIENNAMINI_DETECTINTERFACES_HPP

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

#include "viennamini/forwards.h"

#include "viennagrid/algorithm/interface.hpp"

namespace viennamini {

namespace detail {

template <typename SegmentedMeshT, typename SegmentT, typename IndexContT>
int find_adjacent_segment(SegmentedMeshT& segmesh, SegmentT & current_contact_segment, IndexContT & segments_under_test)
{
  typedef typename viennagrid::result_of::facet<SegmentT>::type                           FacetType;
  typedef typename viennagrid::result_of::const_facet_range<SegmentT>::type               ConstFacetSegmentRangeType;
  typedef typename viennagrid::result_of::iterator<ConstFacetSegmentRangeType>::type      ConstFacetSegmentIteratorType;

  ConstFacetSegmentRangeType const& facets = viennagrid::elements<FacetType>(current_contact_segment);

  // segments under test: these are either all oxide or semiconductor segments
  //
  for(typename IndexContT::iterator sit = segments_under_test.begin();
      sit != segments_under_test.end(); sit++)
  {
    typename SegmentedMeshT::segmentation_type::segment_handle_type current_segment = segmesh.segmentation[*sit];

    for (ConstFacetSegmentIteratorType fit = facets.begin(); fit != facets.end(); ++fit)
    {
      if (viennagrid::is_interface(current_contact_segment, current_segment, *fit))
      {
        return *sit;
      }
    }
  }
  return -1; // not found ..
}

template<typename SegmentedMeshT, typename IndexContT, typename IndexMapT>
void detect_interfaces_impl(SegmentedMeshT& segmesh, IndexContT& contacts, IndexContT& oxides, IndexContT& semiconductors, 
                            IndexMapT& contactSemiconductorInterfaces, IndexMapT& contactOxideInterfaces)
{
  // traverse only contact segments
  // for each contact segment, determine whether it shares an interface with an oxide or a semiconductor
  //
  for(typename IndexContT::iterator cs_it = contacts.begin();
      cs_it != contacts.end(); cs_it++)
  {
    //std::cout << "  * contact-segment " << *cs_it << " : looking for interfaces .." << std::endl;
    typename SegmentedMeshT::segmentation_type::segment_handle_type current_contact_segment = segmesh.segmentation[*cs_it];

    int adjacent_semiconduct_segment_id = viennamini::detail::find_adjacent_segment(segmesh, current_contact_segment, semiconductors);
    if(adjacent_semiconduct_segment_id != -1) // found ..
    {
      //std::cout << "Found neighbour Semiconductor segment #" << adjacent_semiconduct_segment_id << " for contact segment #" << *cs_it << std::endl;
      contactSemiconductorInterfaces[*cs_it] = adjacent_semiconduct_segment_id;
    }
    // if it's not a contact-semiconductor interface -> try a contact-insulator interface
    int adjacent_oxide_segment_id = viennamini::detail::find_adjacent_segment(segmesh, current_contact_segment, oxides);
    if(adjacent_oxide_segment_id != -1) // found ..
    {
      //std::cout << "Found neighbour Oxide segment #" << adjacent_oxide_segment_id << " for contact segment #" << *cs_it << std::endl;
      contactOxideInterfaces[*cs_it] = adjacent_oxide_segment_id;
    }
  }
}

} // detail

template <typename IndexMapT>
void detect_interfaces(viennamini::device& device, IndexMapT& contactSemiconductorInterfaces, IndexMapT& contactOxideInterfaces)
{
  viennamini::device::indices_type & contacts       = device.contact_segments_indices();
  viennamini::device::indices_type & oxides         = device.oxide_segments_indices();
  viennamini::device::indices_type & semiconductors = device.semiconductor_segments_indices();

  if(device.is_triangular2d())
    viennamini::detail::detect_interfaces_impl(device.get_segmesh_triangular_2d(), contacts, oxides, semiconductors, contactSemiconductorInterfaces, contactOxideInterfaces);
  else 
  if(device.is_tetrahedral3d())
    viennamini::detail::detect_interfaces_impl(device.get_segmesh_tetrahedral_3d(), contacts, oxides, semiconductors, contactSemiconductorInterfaces, contactOxideInterfaces);
  else
    std::cout << "detect_interfaces: segmented mesh type not supported" << std::endl;
}

} // viennamini

#endif


#ifndef VIENNAMINI_POSTPROCESSING_HPP
#define VIENNAMINI_POSTPROCESSING_HPP

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
#include "viennagrid/algorithm/volume.hpp"

namespace viennamini {

template<typename SegmentT, typename QuantityT>
double get_terminal_current(SegmentT        & terminal,
                            SegmentT        & semiconductor,
                            QuantityT  const& current_density)
{
  typedef typename viennagrid::result_of::cell<SegmentT>::type                 CellType;
  typedef typename viennagrid::result_of::facet<SegmentT>::type                FacetType;
  typedef typename viennagrid::result_of::facet_range<SegmentT>::type          FacetRange;
  typedef typename viennagrid::result_of::iterator<FacetRange>::type           FacetIterator;

  typedef typename viennagrid::result_of::coboundary_range<
    SegmentT, FacetType, CellType>::type                                       CoboundaryRangeType;

  double current = 0;

  FacetRange facets = viennagrid::elements<FacetType>(terminal);
  for (FacetIterator fit = facets.begin(); fit != facets.end(); ++fit)
  {
    FacetType & facet = *fit;
    if (viennagrid::is_interface(terminal, semiconductor, facet))
    {
      CoboundaryRangeType coboundary_range(semiconductor, fit.handle());
//      std::cout << " facet area:           " << viennagrid::volume(facet) << std::endl;
//      std::cout << " cell current density: " << current_density.get_value(coboundary_range.front()) << std::endl;
      current += current_density.get_value(coboundary_range.front()) * viennagrid::volume(facet);
    }
  }
  return current;
}

template<typename SegmentT, typename QuantityT>
double get_terminal_current(SegmentT       & terminal,
                            SegmentT       & semiconductor,
                            QuantityT const& electron_density,
                            QuantityT const& hole_density)
{
  typedef typename viennagrid::result_of::cell<SegmentT>::type                 CellType;
  typedef typename viennagrid::result_of::facet<SegmentT>::type                FacetType;
  typedef typename viennagrid::result_of::facet_range<SegmentT>::type          FacetRange;
  typedef typename viennagrid::result_of::iterator<FacetRange>::type           FacetIterator;

  typedef typename viennagrid::result_of::coboundary_range<
    SegmentT, FacetType, CellType>::type                                       CoboundaryRangeType;

  double current = 0;
//  std::size_t cnt = 0;
  FacetRange facets = viennagrid::elements<FacetType>(terminal);
  for (FacetIterator fit = facets.begin(); fit != facets.end(); ++fit)
  {
    FacetType & facet = *fit;
    if (viennagrid::is_interface(terminal, semiconductor, facet))
    {
//      cnt++;
      CoboundaryRangeType coboundary_range(semiconductor, fit.handle());
      std::cout << " A:  " << viennagrid::volume(facet) << std::endl;
      std::cout << " Jn: " << electron_density.get_value(coboundary_range.front()) << std::endl;
      std::cout << " Jp: " << hole_density.get_value(coboundary_range.front()) << std::endl;
      std::cout << " I:  " << (electron_density.get_value(coboundary_range.front()) + hole_density.get_value(coboundary_range.front())) * viennagrid::volume(facet) << std::endl;
      // I = (Jn + Jp) * A
      current += (electron_density.get_value(coboundary_range.front()) + hole_density.get_value(coboundary_range.front())) * viennagrid::volume(facet);
    }
  }
//  std::cout << "contact edges: " << cnt << std::endl;
  return current;
}

} // viennamini

#endif


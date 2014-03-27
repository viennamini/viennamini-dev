#ifndef VIENNAMINI_CONTACTMODELS_OHMICCONTACT_HPP
#define VIENNAMINI_CONTACTMODELS_OHMICCONTACT_HPP

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

#include "viennamini/forwards.h"
#include "viennamini/physics.hpp"
#include "viennamini/contact_model.hpp"

#include "viennagrid/algorithm/interface.hpp"

namespace viennamini {

class ohmic_contact : public contact_model
{
public:
  void apply(viennamini::device_handle& device, std::size_t segment_index)
  {
    // extract the required quantities from the neighbour segment
    //
    quantity_set qset;
    if(device->is_line1d()) 
      qset = extract_quantities(device->get_segmesh_line_1d(), device, segment_index);
    else
    if(device->is_triangular2d())
      qset = extract_quantities(device->get_segmesh_triangular_2d(), device, segment_index);
    else
    if(device->is_tetrahedral3d())
      qset = extract_quantities(device->get_segmesh_tetrahedral_3d(), device, segment_index);

    // apply the individual, quantity-specific contact models
    //
    if(get_quantity_name() == viennamini::id::potential())
    {
      device->set_contact(get_quantity_name(), segment_index, device->get_contact(get_quantity_name(), segment_index) + viennamini::built_in_potential(qset.ND, qset.NA, qset.T, qset.ni));
    }
    else 
    if(get_quantity_name() == viennamini::id::electron_concentration())
    {
      device->set_contact(get_quantity_name(), segment_index, viennamini::ohmic_electrons_initial(qset.ND, qset.NA, qset.ni));
    }
    else 
    if(get_quantity_name() == viennamini::id::hole_concentration())
    {
      device->set_contact(get_quantity_name(), segment_index, viennamini::ohmic_holes_initial(qset.ND, qset.NA, qset.ni));
    }
    else throw contact_model_exception("Ohmic contact model is not defined for quantity \""+get_quantity_name()+"\"!");
  }

private:

  struct quantity_set
  {
    viennamini::numeric ND, NA, T, ni;
  };


  template<typename SegMeshT>
  quantity_set extract_quantities(SegMeshT& segmesh, viennamini::device_handle& device, std::size_t segment_index)
  {
    typedef typename SegMeshT::segmentation_type                                    SegmentationType;
    typedef typename SegmentationType::segment_handle_type                          SegmentType;
    typedef typename viennagrid::result_of::facet_range<SegmentationType>::type     FacetRange;
    typedef typename viennagrid::result_of::iterator<FacetRange>::type              Facetterator;
    typedef typename viennagrid::result_of::facet<SegmentationType>::type           FacetType;
    typedef typename viennagrid::result_of::cell_tag<SegmentationType>::type        CellTag;
    typedef typename viennagrid::result_of::facet_tag<SegmentationType>::type       FacetTag;

    typedef typename viennagrid::result_of::coboundary_range<SegmentType, FacetTag, CellTag>::type     CellOnFacetRange;
    typedef typename viennagrid::result_of::iterator<CellOnFacetRange>::type                           CellOnFacetIterator;

    std::size_t semiconductor_segment = device->get_adjacent_semiconductor_segment_for_contact(segment_index);

    SegmentType seg_contact_handle       = segmesh.segmentation[segment_index];
    SegmentType seg_semiconductor_handle = segmesh.segmentation[semiconductor_segment];

    FacetRange facets = viennagrid::elements<FacetType>(seg_semiconductor_handle);
    for(Facetterator fit = facets.begin(); fit != facets.end(); fit++)
    {
      // at the interface ..
      if(viennagrid::is_interface(seg_contact_handle, seg_semiconductor_handle, *fit))
      {
        CellOnFacetRange cells_on_facet = viennagrid::coboundary_elements<FacetType, CellTag>(seg_semiconductor_handle, fit.handle());
        if(cells_on_facet.size() != 1) throw contact_model_exception("Ohmic contact model failed to identify the interface!");
        for (CellOnFacetIterator cofit = cells_on_facet.begin(); cofit != cells_on_facet.end(); ++cofit)
        {
          quantity_set qset;
          qset.ND = device->get_quantity(viennamini::id::donor_doping(),      semiconductor_segment, (*cofit).id().get());
          qset.NA = device->get_quantity(viennamini::id::acceptor_doping(),   semiconductor_segment, (*cofit).id().get());
          qset.T  = device->get_quantity(viennamini::id::temperature(),       semiconductor_segment, (*cofit).id().get());
          qset.ni = device->get_quantity(viennamini::id::intrinsic_carrier(), semiconductor_segment, (*cofit).id().get());
          return qset; // we only need the quantity from the first cell we find, as all cells along the interface on the semiconductor 
          // segment are assumed to offer constant quantitiy values of ND, NA, ni, T
        }
      }
    }
    throw contact_model_exception("Ohmic contact model failed to evaluate the builtin potential for the interface!");
    return quantity_set(); // silence the compiler
  }

};

} // viennamini


#endif


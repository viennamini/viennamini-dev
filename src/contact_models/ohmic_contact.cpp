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

#include "viennamini/contact_models/ohmic_contact.hpp"

namespace viennamini {

void ohmic_contact::apply(viennamini::device_handle& device, std::size_t segment_index)
{
    // the ohmic contact model does not make sense for a metal-oxide interface
    if(device->is_contact_at_oxide(segment_index)) return;

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

} // viennamini


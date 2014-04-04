#ifndef VIENNAMINI_GENERATEPDESET_HPP
#define VIENNAMINI_GENERATEPDESET_HPP

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

#include "viennamini/pde_sets/laplace.hpp"
#include "viennamini/pde_sets/drift_diffusion.hpp"

namespace viennamini {

viennamini::pde_set_handle generate_pde_set(viennamini::pdeset::pdeset_ids pde_set_id);

} // viennamini

#endif


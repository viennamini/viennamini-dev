#ifndef VIENNAMINI_PDESETS_DRIFTDIFFUSION_HPP
#define VIENNAMINI_PDESETS_DRIFTDIFFUSION_HPP

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
#include "viennamini/pde_set.hpp"
#include "viennamini/constants.hpp"
#include "viennamini/quantity_generators/builtin_potential.hpp"
#include "viennamini/quantity_generators/use_quantity.hpp"
#include "viennamini/contact_models/ohmic_contact.hpp"

namespace viennamini {

class drift_diffusion : public viennamini::pde_set
{
public:

  drift_diffusion();

  std::string info();

  pdes_type get_pdes();

  bool is_linear();

};

} // viennamini


#endif


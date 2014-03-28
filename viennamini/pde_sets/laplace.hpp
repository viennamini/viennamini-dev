#ifndef VIENNAMINI_PDESETS_LAPLACE_HPP
#define VIENNAMINI_PDESETS_LAPLACE_HPP

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

namespace viennamini {

class laplace : public viennamini::pde_set
{
public:

  laplace();

  std::string info();

  pdes_type get_pdes();

  bool is_linear();

};

} // viennamini


#endif


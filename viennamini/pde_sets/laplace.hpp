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

  laplace()  
  {
    add_dependency(viennamini::id::permittivity());
//    add_unknown(viennamini::id::potential());

//    function_symbols_.push_back(viennamath::function_symbol(0));
//    function_symbols_.push_back(viennamath::function_symbol(1));

//    viennamath::function_symbol psi (potential.id());
//    viennamath::function_symbol eps (permittivity.id());

//    equation_ = viennamath::make_equation( viennamath::div(eps * viennamath::grad(psi)), /* = */ 0);
  }

  ~laplace() {}

  std::string info()
  {
    return "The Laplace equation with meta information.";
  }

};

} // viennamini


#endif


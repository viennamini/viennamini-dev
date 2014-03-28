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

namespace viennamini {

laplace::laplace()
{
  add_dependency(viennamini::id::permittivity());
  add_role_support(viennamini::id::permittivity(),            viennamini::role::oxide);
  add_role_support(viennamini::id::permittivity(),            viennamini::role::semiconductor);

  add_unknown(viennamini::id::potential());
  add_role_support(viennamini::id::potential(), viennamini::role::oxide);
  add_role_support(viennamini::id::potential(), viennamini::role::semiconductor);
}

std::string laplace::info()
{
  return "The Laplace equation.";
}

laplace::pdes_type laplace::get_pdes()
{
  FunctionSymbolType  eps(get_quantity_id(viennamini::id::permittivity()));
  FunctionSymbolType  psi(get_quantity_id(viennamini::id::potential()));

  pdes_type pdes;

  pdes.push_back(
    viennamini::pde(
      viennamath::make_equation( viennamath::div(eps * viennamath::grad(psi)), /* = */ 0),
      psi,
      viennamath::expr(viennamath::rt_constant<double>(0)),
      false
    )
  );
  return pdes;
}

bool laplace::is_linear()
{
  return true;
}

} // viennamini


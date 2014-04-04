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

#include "viennamini/pde_sets/drift_diffusion.hpp"
#include "viennamini/constants.hpp"
#include "viennamini/quantity_generators/builtin_potential.hpp"
#include "viennamini/quantity_generators/use_quantity.hpp"
#include "viennamini/contact_models/ohmic_contact.hpp"

namespace viennamini {

drift_diffusion::drift_diffusion()
{
  add_dependency(viennamini::id::relative_permittivity());
  add_dependency(viennamini::id::temperature());
  add_dependency(viennamini::id::thermal_potential());
  add_dependency(viennamini::id::donor_doping());
  add_dependency(viennamini::id::acceptor_doping());
  add_dependency(viennamini::id::electron_mobility());
  add_dependency(viennamini::id::hole_mobility());
  add_dependency(viennamini::id::intrinsic_carrier());

  add_role_support(viennamini::id::relative_permittivity(),   viennamini::role::contact);
  add_role_support(viennamini::id::relative_permittivity(),   viennamini::role::oxide);
  add_role_support(viennamini::id::relative_permittivity(),   viennamini::role::semiconductor);
  add_role_support(viennamini::id::temperature(),             viennamini::role::semiconductor);
  add_role_support(viennamini::id::thermal_potential(),       viennamini::role::semiconductor);
  add_role_support(viennamini::id::thermal_potential(),       viennamini::role::oxide); // VT is used in the damping term of the Poisson equation
  add_role_support(viennamini::id::donor_doping(),            viennamini::role::semiconductor);
  add_role_support(viennamini::id::acceptor_doping(),         viennamini::role::semiconductor);
  add_role_support(viennamini::id::electron_mobility(),       viennamini::role::semiconductor);
  add_role_support(viennamini::id::hole_mobility(),           viennamini::role::semiconductor);
  add_role_support(viennamini::id::intrinsic_carrier(),       viennamini::role::semiconductor);

  add_unknown(viennamini::id::potential());
  add_unknown(viennamini::id::electron_concentration());
  add_unknown(viennamini::id::hole_concentration());

  add_role_support(viennamini::id::potential(),               viennamini::role::oxide);
  add_role_support(viennamini::id::potential(),               viennamini::role::semiconductor);
  add_role_support(viennamini::id::electron_concentration(),  viennamini::role::semiconductor);
  add_role_support(viennamini::id::hole_concentration(),      viennamini::role::semiconductor);

  set_initial_guess(viennamini::id::potential(),              new viennamini::builtin_potential());
  set_initial_guess(viennamini::id::electron_concentration(), new viennamini::use_quantity(viennamini::id::donor_doping()));
  set_initial_guess(viennamini::id::hole_concentration(),     new viennamini::use_quantity(viennamini::id::acceptor_doping()));

  set_contact_model(viennamini::id::potential(),              new viennamini::ohmic_contact());
  set_contact_model(viennamini::id::electron_concentration(), new viennamini::ohmic_contact());
  set_contact_model(viennamini::id::hole_concentration(),     new viennamini::ohmic_contact());
}

std::string drift_diffusion::info()
{
  return "The drift-diffusion transport model, consisting of Poisson's equation coupled with the drift-diffusion-modeled continuity equation for electrons and holes.";
}

drift_diffusion::pdes_type drift_diffusion::get_pdes()
{
  FunctionSymbolType  epsr(get_quantity_id(viennamini::id::relative_permittivity()));
  FunctionSymbolType  VT  (get_quantity_id(viennamini::id::thermal_potential()));
  FunctionSymbolType  ND  (get_quantity_id(viennamini::id::donor_doping()));
  FunctionSymbolType  NA  (get_quantity_id(viennamini::id::acceptor_doping()));
  FunctionSymbolType  mu_n(get_quantity_id(viennamini::id::electron_mobility()));
  FunctionSymbolType  mu_p(get_quantity_id(viennamini::id::hole_mobility()));
  FunctionSymbolType  psi (get_quantity_id(viennamini::id::potential()));
  FunctionSymbolType  n   (get_quantity_id(viennamini::id::electron_concentration()));
  FunctionSymbolType  p   (get_quantity_id(viennamini::id::hole_concentration()));

  pdes_type pdes;

  pdes.push_back(
    viennamini::pde(
      viennamath::make_equation( viennamath::div(epsr * viennamini::eps0::val()  * viennamath::grad(psi)), /* = */ viennamini::q::val() * ((n - ND) - (p - NA))),
      psi,
      viennamath::expr( (n + p) * (-viennamini::q::val() / VT) ), // let n ~ exp(psi/VT), p~exp(-psi/VT)     dn/dpsi = n * q / VT,    dp/dpsi = - p * q / Vt
      false
    )
  );

  pdes.push_back(
    viennamini::pde(
      viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(n) - mu_n * viennamath::grad(psi) * n) , /* = */ 0.0),
      n,
      viennamath::expr(viennamath::rt_constant<double>(0)),
      true
    )
  );

  pdes.push_back(
    viennamini::pde(
      viennamath::make_equation( viennamath::div(mu_p * VT * viennamath::grad(p) + mu_p * viennamath::grad(psi) * p) , /* = */ 0.0),
      p,
      viennamath::expr(viennamath::rt_constant<double>(0)),
      true
    )
  );

  return pdes;
}

bool drift_diffusion::is_linear()
{
  return false;
}

} // viennamini



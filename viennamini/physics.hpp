
#ifndef VIENNAMINI_PHYSICS_HPP
#define VIENNAMINI_PHYSICS_HPP

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

#include "viennamini/constants.hpp"

namespace viennamini
{
  // TODO http://ecee.colorado.edu/~bart/book/ex019.htm


  // ===========================================================================
  //
  // General
  //
  // ===========================================================================

  // ---------------------------------------------------------------------------
  //
  // Thermal Potential
  //

  template<typename NumericT>
  inline NumericT thermal_potential(NumericT T)
  {
    return (viennamini::kB::val() * T) / viennamini::q::val();
  }

  // ---------------------------------------------------------------------------
  //
  // Builtin Potential
  //

  template<typename NumericT>
  inline NumericT arsinh(NumericT x)
  {
    return std::log(x + sqrt((x * x) + 1.0));
  }

  template<typename NumericT>
  inline NumericT built_in_potential(NumericT const& ND, NumericT const& NA, NumericT const& T, NumericT const& ni)
  {
    return viennamini::thermal_potential(T) * arsinh(0.5 * (ND-NA)/ni);
  }

  // ---------------------------------------------------------------------------
  //
  // Carrier Lifetimes
  //

  template<typename NumericT>
  inline NumericT carrier_lifetimes(NumericT const& ND, NumericT const& NA, NumericT const& N_ref, NumericT const& tau_0)
  {
    return ( tau_0 / ( 1 + (NA + ND) / N_ref ) );
  }

  // ---------------------------------------------------------------------------
  //
  // Ohmic Electrons/Holes Initial Guess
  //

  template<typename NumericT>
  inline NumericT ohmic_electrons_initial(NumericT const& ND, NumericT const& NA, NumericT const& ni)
  {
    return 0.5*(std::sqrt((ND-NA)*(ND-NA)+4.0*ni*ni)+(ND-NA));
  }

  template<typename NumericT>
  inline NumericT ohmic_holes_initial(NumericT const& ND, NumericT const& NA, NumericT const& ni)
  {
    return 0.5*(std::sqrt((ND-NA)*(ND-NA)+4.0*ni*ni)-(ND-NA));
  }

  // ===========================================================================
  //
  // Mobility Models
  //
  // ===========================================================================

  namespace mobility {

    // ---------------------------------------------------------------------------
    //
    // Lattice Scattering
    //

    template<typename NumericT>
    inline NumericT lattice_scattering(NumericT const& mu_0, NumericT const& alpha, NumericT const& T)
    {
      return mu_0 * std::pow(T/300., (-1.0)*alpha);
    }

    // ---------------------------------------------------------------------------
    //
    // Ionizied Impurity Scattering
    //

    template<typename NumericT>
    inline NumericT ionized_impurity_scattering(NumericT const& mu_lattice, NumericT const& mu_min, NumericT const& alpha, NumericT const& N_I, NumericT const& N_ref)
    {
      return mu_min + (mu_lattice-mu_min)/(1+std::pow(N_I/N_ref, alpha));
    }

  } // mobility

} // viennamini



#endif


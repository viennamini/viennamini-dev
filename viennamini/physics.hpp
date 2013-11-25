
#ifndef VIENNAMINI_PHYSICS_HPP
#define VIENNAMINI_PHYSICS_HPP

/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
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
  inline NumericT thermal_potential_impl(NumericT T)
  {
    return (viennamini::kB::val() * T) / viennamini::q::val();
  }

  template<typename QuantityT>
  struct thermal_potential
  {
    typedef viennamini::numeric numeric_type;
    typedef numeric_type        result_type;
    
    thermal_potential(QuantityT& T) :
     T_(T) {}
    
    template<typename CellT>
    result_type operator()(CellT const& cell) 
    {
      return thermal_potential_impl(T_.get_value(cell));
    }
    
  private:
    QuantityT& T_;
  };
  

  // ---------------------------------------------------------------------------
  //
  // Builtin Potential
  //

  template<typename NumericT>
  inline NumericT built_in_potential_impl(NumericT const& ND, NumericT const& NA, NumericT const& T, NumericT const& ni)
  {
    const NumericT net_doping = ND - NA;
    const NumericT x = std::abs(net_doping) / (2.0 * ni);

    NumericT bpot = viennamini::thermal_potential_impl(T) * std::log(x + std::sqrt( 1.0 + x*x ) );

    if ( net_doping < 0) //above formula does not yet consider the doping polarity
      bpot *= -1.0;

    return bpot;
  }
  
  template<typename QuantityT>
  struct built_in_potential
  {
    typedef viennamini::numeric numeric_type;
    typedef numeric_type        result_type;
    
    built_in_potential(QuantityT& ND, QuantityT& NA, QuantityT& ni, QuantityT& T) :
     ND_(ND), NA_(NA), ni_(ni), T_(T) {}
    
    template<typename CellT>
    result_type operator()(CellT const& cell) 
    {
      return built_in_potential_impl(ND_.get_value(cell), NA_.get_value(cell), T_.get_value(cell), ni_.get_value(cell));
    }
    
  private:
    QuantityT& ND_;
    QuantityT& NA_;
    QuantityT& ni_;
    QuantityT& T_;
  };
  
  // ---------------------------------------------------------------------------
  //
  // Carrier Lifetimes
  //

  template<typename NumericT>
  inline NumericT carrier_lifetimes_impl(NumericT const& ND, NumericT const& NA, NumericT const& N_ref, NumericT const& tau_0)
  {
    return ( tau_0 / ( 1 + (NA + ND) / N_ref ) );
  }

  template<typename QuantityT>
  struct carrier_lifetimes
  {
    typedef viennamini::numeric numeric_type;
    typedef numeric_type        result_type;

    carrier_lifetimes(QuantityT& ND, QuantityT& NA, numeric_type const& N_ref, numeric_type const& tau_0) : 
      ND_(ND), NA_(NA), N_ref_(N_ref), tau_0_(tau_0)  {}

    template<typename CellT>
    result_type operator()(CellT const& cell) 
    {
      return carrier_lifetimes_impl(ND_.get_value(cell), NA_.get_value(cell), N_ref_, tau_0_);
    }

    QuantityT   & ND_;
    QuantityT   & NA_; 
    numeric_type  N_ref_; 
    numeric_type  tau_0_;
  };

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
    inline NumericT lattice_scattering_impl(NumericT const& mu_0, NumericT const& alpha, NumericT const& T)
    {
      return mu_0 * std::pow(T/300., (-1.0)*alpha);
    }

    template<typename QuantityT>
    struct lattice_scattering
    {
      typedef viennamini::numeric numeric_type;
      typedef numeric_type        result_type;
      
      lattice_scattering(numeric_type const& mu_0, numeric_type const& alpha, QuantityT& T) : mu_0_(mu_0), alpha_(alpha), T_(T) {}
      
      template<typename CellT>
      result_type operator()(CellT const& cell) 
      {
        return lattice_scattering_impl(mu_0_, alpha_, T_.get_value(cell));
      }
      
    private:
      numeric_type    mu_0_;
      numeric_type    alpha_;
      QuantityT & T_;
    };

    // ---------------------------------------------------------------------------
    //
    // Ionizied Impurity Scattering
    //

    template<typename NumericT>
    inline NumericT ionized_impurity_scattering_impl(NumericT const& mu_lattice, NumericT const& mu_min, NumericT const& alpha, NumericT const& N_I, NumericT const& N_ref)
    {
      return mu_min + (mu_lattice-mu_min)/(1+std::pow(N_I/N_ref, alpha));
    }

    template<typename FunctorT, typename QuantityT>
    struct ionized_impurity_scattering
    {
      typedef viennamini::numeric numeric_type;
      typedef numeric_type        result_type;
      
      ionized_impurity_scattering(FunctorT mu_lattice, QuantityT& ND, QuantityT& NA, numeric_type const& alpha, numeric_type const& mu_min, numeric_type const& N_ref) 
        : mu_lattice_(mu_lattice), ND_(ND), NA_(NA), alpha_(alpha), mu_min_(mu_min), N_ref_(N_ref) {}
      
      template<typename CellT>
      result_type operator()(CellT const& cell) 
      {
        return ionized_impurity_scattering_impl(mu_lattice_(cell), mu_min_, alpha_, ND_.get_value(cell)+NA_.get_value(cell), N_ref_);
      }
      
    private:
      FunctorT       mu_lattice_;
      QuantityT &    ND_;
      QuantityT &    NA_;
      numeric_type   alpha_;
      numeric_type   mu_min_;
      numeric_type   N_ref_;
    };
  } // mobility

} // viennamini



#endif


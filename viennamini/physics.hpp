
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


  /** @brief Returns the thermal potential for the provided temperature */
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
  



} // viennamini



#endif

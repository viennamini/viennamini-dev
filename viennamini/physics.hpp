
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
  inline NumericT get_thermal_potential(NumericT T)
  {
    return (viennamini::kB::val() * T) / viennamini::q::val();
  }

  template<typename NumericT>
  inline NumericT built_in_potential(NumericT temp, NumericT doping_n, NumericT doping_p)
  {
    const NumericT ni = 1.e16; // TODO!!
    const NumericT net_doping = doping_n - doping_p;
    const NumericT x = std::abs(net_doping) / (2.0 * ni);

    NumericT bpot = viennamini::get_thermal_potential(temp) * std::log(x + std::sqrt( 1.0 + x*x ) );

    if ( net_doping < 0) //above formula does not yet consider the doping polarity
      bpot *= -1.0;

    return bpot;
  }
  


} // viennamini



#endif

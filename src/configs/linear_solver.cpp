/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */

#include "viennamini/configs/linear_solver.hpp"


namespace viennamini {
namespace config {

  linear_solver::linear_solver()
    : iterations_(1000),
      breaktol_(1.E-14)
  {
  }

  std::size_t& linear_solver::iterations()
  {
    return iterations_;
  }

  viennamini::numeric& linear_solver::breaktol()
  {
    return breaktol_;
  }

} // config
} // viennamini



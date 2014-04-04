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

#include "viennamini/configs/nonlinear_solver.hpp"


namespace viennamini {
namespace config {

  nonlinear_solver::nonlinear_solver()
    : iterations_(100),
      breaktol_(1.E-3),
      damping_(0.9)
  {
  }

  std::size_t&          nonlinear_solver::iterations()
  {
    return iterations_;
  }

  viennamini::numeric&  nonlinear_solver::breaktol()
  {
    return breaktol_;
  }

  viennamini::numeric&  nonlinear_solver::damping()
  {
    return damping_;
  }

} // config
} // viennamini



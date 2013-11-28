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


#include "viennamini/config.hpp"


namespace viennamini {

config::config(std::ostream& stream) : stream_(stream)
{
  nonlinear_iterations_                = 100;
  nonlinear_breaktol_                  = 1.E-3;
  linear_breaktol_                     = 1.E-14;
  linear_iterations_                   = 1000;
  damping_                             = 0.9;
  write_initial_guesses_               = true;
  write_simulation_results_            = true;
}


config::IndexType&    config::nonlinear_iterations()
{
  return nonlinear_iterations_;
}

config::NumericType&  config::nonlinear_breaktol()
{
  return nonlinear_breaktol_;
}

config::IndexType&    config::linear_iterations()
{
  return linear_iterations_;
}

config::NumericType&  config::linear_breaktol()
{
  return linear_breaktol_;
}

config::NumericType&  config::damping()
{
  return damping_;
}

bool& config::write_initial_guess_files()
{
  return write_initial_guesses_;
}

bool& config::write_result_files()
{
  return write_simulation_results_;
}

std::ostream& config::stream()
{
  return stream_;
}

} // viennamini


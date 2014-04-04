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


#include "viennamini/configuration.hpp"


namespace viennamini {

configuration::configuration(std::ostream& stream) : stream_(stream)
{
  write_initial_guesses_               = true;
  write_simulation_results_            = true;
}

bool& configuration::write_initial_guess_files()
{
  return write_initial_guesses_;
}

bool& configuration::write_result_files()
{
  return write_simulation_results_;
}

std::ostream& configuration::stream()
{
  return stream_;
}

configuration::linear_solver_config_type & configuration::linear_solver()
{
  return linear_solver_;
}

configuration::nonlinear_solver_config_type & configuration::nonlinear_solver()
{
  return nonlinear_solver_;
}

configuration::ModelType& configuration::model()
{
  return model_;
}

} // viennamini


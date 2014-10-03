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
  nonlinear_iterations_                = 100;
  nonlinear_breaktol_                  = 1.E-3;
  linear_breaktol_                     = 1.E-14;
  linear_iterations_                   = 1000;
  damping_                             = 0.9;
  write_initial_guesses_               = true;
  write_simulation_results_            = true;
  result_filename_                     = "output";
  initial_guess_filename_              = "initial";
}

configuration::IndexType&    configuration::nonlinear_iterations()
{
  return nonlinear_iterations_;
}

configuration::NumericType&  configuration::nonlinear_breaktol()
{
  return nonlinear_breaktol_;
}

configuration::IndexType&    configuration::linear_iterations()
{
  return linear_iterations_;
}

configuration::NumericType&  configuration::linear_breaktol()
{
  return linear_breaktol_;
}

configuration::NumericType&  configuration::damping()
{
  return damping_;
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

configuration::ModelType&   configuration::model()
{
  return model_;
}

std::string&  configuration::result_filename()
{
  return result_filename_;
}

std::string&  configuration::initial_guess_filename()
{
  return initial_guess_filename_;
}

} // viennamini

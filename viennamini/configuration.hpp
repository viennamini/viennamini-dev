#ifndef VIENNAMINI_CONFIGURATION_HPP
#define VIENNAMINI_CONFIGURATION_HPP

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

#include <iostream>
#include <map>
#include <vector>

#include "viennamini/forwards.h"
#include "viennamini/configs/model.hpp"
#include "viennamini/configs/linear_solver.hpp"
#include "viennamini/configs/nonlinear_solver.hpp"

namespace viennamini {

struct configuration
{
private:
  typedef viennamini::config::linear_solver     LinearSolverConfigType;
  typedef viennamini::config::nonlinear_solver  NonLinearSolverConfigType;
  typedef viennamini::config::model             ModelType;

public:
  typedef LinearSolverConfigType                linear_solver_config_type;
  typedef NonLinearSolverConfigType             nonlinear_solver_config_type;
  typedef ModelType                             model_type;

  configuration(std::ostream& stream = std::cout);

  bool&         write_initial_guess_files();
  bool&         write_result_files();
  std::ostream& stream();

  linear_solver_config_type     & linear_solver();
  nonlinear_solver_config_type  & nonlinear_solver();
  model_type                    & model();

private:


  bool              write_initial_guesses_;
  bool              write_simulation_results_;
  std::ostream&     stream_;

  LinearSolverConfigType    linear_solver_;
  NonLinearSolverConfigType nonlinear_solver_;
  ModelType                 model_;
};




} // viennamini



#endif



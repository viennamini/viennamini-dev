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


namespace viennamini {

struct configuration
{
private:
  typedef viennamini::numeric                 NumericType;
  typedef int                                 IndexType;
  typedef std::vector<NumericType>            ValuesType;
  typedef std::map<std::size_t, NumericType>  SegmentValuesType;
  typedef viennamini::config::model           ModelType;

public:
  typedef NumericType       numeric_type;
  typedef IndexType         index_type;
  typedef ValuesType        values_type;
  typedef SegmentValuesType segmentvalues_type;
  typedef ModelType         model_type;

  configuration(std::ostream& stream = std::cout);

  IndexType&    nonlinear_iterations();
  NumericType&  nonlinear_breaktol();
  IndexType&    linear_iterations();
  NumericType&  linear_breaktol();
  NumericType&  damping();
  bool&         write_initial_guess_files();
  bool&         write_result_files();
  std::ostream& stream();
  ModelType&    model();
  std::string&  result_filename();
  std::string&  initial_guess_filename();

private:
  IndexType         nonlinear_iterations_;
  IndexType         linear_iterations_;
  NumericType       nonlinear_breaktol_;
  NumericType       linear_breaktol_;
  NumericType       damping_;
  bool              write_initial_guesses_;
  bool              write_simulation_results_;
  std::ostream&     stream_;
  ModelType         model_;
  std::string       result_filename_;
  std::string       initial_guess_filename_;
};




} // viennamini



#endif

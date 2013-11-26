#ifndef VIENNAMINI_CONFIG_HPP
#define VIENNAMINI_CONFIG_HPP

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

namespace viennamini {

struct config
{
  typedef viennamini::numeric                 NumericType;
  typedef int                                 IndexType;
  typedef std::vector<NumericType>            ValuesType;
  typedef std::map<std::size_t, NumericType>  SegmentValuesType;

  typedef NumericType       numeric_type;
  typedef IndexType         index_type;
  typedef ValuesType        values_type;
  typedef SegmentValuesType segmentvalues_type;

  config(std::ostream& stream = std::cout);

  NumericType&  temperature();
  IndexType&    nonlinear_iterations();
  NumericType&  nonlinear_breaktol();
  IndexType&    linear_iterations();
  NumericType&  linear_breaktol();
  NumericType&  damping();
  IndexType&    initial_guess_smoothing_iterations();
  bool&         write_initial_guesses();
  std::string&  problem();
  std::ostream& stream();
  
private:
  IndexType         nonlinear_iterations_;
  IndexType         linear_iterations_;
  IndexType         initial_guess_smoothing_iterations_;
  NumericType       temperature_;
  NumericType       nonlinear_breaktol_;
  NumericType       linear_breaktol_;
  NumericType       damping_;
  std::string       problem_;
  bool              write_initial_guesses_;
  std::ostream&     stream_;
};




} // viennamini



#endif



#ifndef VIENNAMINI_PROBLEM_HPP
#define VIENNAMINI_PROBLEM_HPP

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

// ViennaMath includes:
#include "viennamath/expression.hpp"

#include "viennamini/forwards.h"

namespace viennamini {

struct problem
{
public:
  typedef boost::variant<null, problem_description_triangular_2d, problem_description_tetrahedral_3d> GenericProblemDescriptionType;
  typedef viennamini::numeric           NumericType;
  typedef viennamath::function_symbol   FunctionSymbolType;
  typedef viennamath::equation          EquationType;
  
  typedef GenericProblemDescriptionType generic_problem_description_type;
  typedef FunctionSymbolType            function_symbol_type;
  typedef EquationType                  equation_type;
  typedef NumericType                   numeric_type;

  problem(viennamini::device& device, viennamini::config& config, viennamini::material_library& matlib) 
    : device_(device), config_(config), matlib_(matlib) 
  {
  }


  virtual void run() = 0;
  virtual void write(std::string const& filename) = 0;
  
  viennamini::device&           device_;
  viennamini::config&           config_;
  viennamini::material_library& matlib_;
  GenericProblemDescriptionType problem_description_;
};

} // viennamini

#endif





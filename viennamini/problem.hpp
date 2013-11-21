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

// ViennaFVM includes:
#ifdef VIENNAMINI_VERBOSE
  #define VIENNAFVM_VERBOSE
#endif
#include "viennafvm/forwards.h"
#include "viennafvm/boundary.hpp"
#include "viennafvm/io/vtk_writer.hpp"

#include "viennamini/forwards.h"

namespace viennamini {


/** @brief Exception for the case that an invalid quantity is accessed */
class quantity_not_found_exception : public std::runtime_error {
public:
  quantity_not_found_exception(std::string const & str) : std::runtime_error(str) {}
};


struct problem
{
public:
  typedef viennamini::numeric           NumericType;
  typedef viennamath::function_symbol   FunctionSymbolType;
  typedef viennamath::equation          EquationType;
  
  typedef FunctionSymbolType            function_symbol_type;
  typedef EquationType                  equation_type;
  typedef NumericType                   numeric_type;

  problem(viennamini::device& device, viennamini::config& config) 
    : device_(device), config_(config) 
  {
  }

  virtual void run() = 0;

  void write(std::string const& filename)
  {
    if(device_.is_triangular2d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device_.get_problem_description_triangular_2d().quantities(), 
        filename, 
        device_.get_segmesh_triangular_2d().mesh, 
        device_.get_segmesh_triangular_2d().segmentation);
    }
    else 
    if(device_.is_tetrahedral3d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device_.get_problem_description_tetrahedral_3d().quantities(), 
        filename, 
        device_.get_segmesh_tetrahedral_3d().mesh, 
        device_.get_segmesh_tetrahedral_3d().segmentation);
    }
  }
  
  viennamini::device&           device_;
  viennamini::config&           config_;
};

} // viennamini

#endif





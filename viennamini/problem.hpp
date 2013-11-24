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
#include "viennafvm/pde_solver.hpp"
#include "viennafvm/problem_description.hpp"
#include "viennafvm/forwards.h"
#include "viennafvm/boundary.hpp"
#include "viennafvm/io/vtk_writer.hpp"

#include "viennamini/forwards.h"
#include "viennamini/physics.hpp"

#include <boost/lexical_cast.hpp>

namespace viennamini {


/** @brief Exception for the case that an invalid quantity is accessed */
class quantity_not_found_exception : public std::runtime_error {
public:
  quantity_not_found_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief Exception for the case that a contact segment could not be identified as either a contact-semiconducotr nor a contact-oxide segment */
class segment_undefined_contact_exception : public std::runtime_error {
public:
  segment_undefined_contact_exception(int segment_index) : std::runtime_error(" at segment: "+boost::lexical_cast<std::string>(segment_index)) {}
};

/** @brief Exception for the case that a segment could not be identified as either a contact, an oxide, nor a semiconductor */
class segment_undefined_exception : public std::runtime_error {
public:
  segment_undefined_exception(int segment_index) : std::runtime_error(" at segment: "+boost::lexical_cast<std::string>(segment_index)) {}
};

/** @brief Exception for the case that a mobility model is not supported */
class mobility_not_supported_exception : public std::runtime_error {
public:
  mobility_not_supported_exception() : std::runtime_error("") {}
};

/** @brief Exception for the case that a recombination model is not supported */
class recombination_not_supported_exception : public std::runtime_error {
public:
  recombination_not_supported_exception() : std::runtime_error("") {}
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
    if(device_.is_line1d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device_.get_problem_description_line_1d().quantities(), 
        filename, 
        device_.get_segmesh_line_1d().mesh, 
        device_.get_segmesh_line_1d().segmentation);
    }
    else
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
    else throw device_not_supported_exception("at: problem::write()"); 
  }
  
  viennamini::device&           device_;
  viennamini::config&           config_;
};

} // viennamini

#endif





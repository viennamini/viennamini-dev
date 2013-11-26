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

#ifndef NDEBUG
  #define NDEBUG
#endif

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
#include "viennamini/csv.hpp"

#include <boost/lexical_cast.hpp>


#define VIENNAMINI_PROBLEM(classname) \
public: \
\
  classname(std::ostream& stream) : viennamini::problem(stream) {} \
\
  void run(segment_values& current_contact_potentials, segment_values& current_contact_workfunctions, std::size_t step_id) \
  {\
    if(device().is_line1d()) \
    {\
      this->run_impl(device().get_segmesh_line_1d(), device().get_problem_description_line_1d_set(), current_contact_potentials, current_contact_workfunctions, step_id); \
    }\
    else \
    if(device().is_triangular2d()) \
    {\
      this->run_impl(device().get_segmesh_triangular_2d(), device().get_problem_description_triangular_2d_set(), current_contact_potentials, current_contact_workfunctions, step_id); \
    }\
    else \
    if(device().is_tetrahedral3d()) \
    {\
      this->run_impl(device().get_segmesh_tetrahedral_3d(), device().get_problem_description_tetrahedral_3d_set(), current_contact_potentials, current_contact_workfunctions, step_id); \
    }\
    else throw device_not_supported_exception("at: problem_laplace::run()"); \
  }

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

  problem(std::ostream& stream = std::cout) : stream_(stream)
  {
  }

  void set(viennamini::device_handle& device_handle, 
           viennamini::config_handle& config_handle)
  {
    device_handle_ = device_handle;
    config_handle_ = config_handle;
  }

  viennamini::device & device()
  {
    return *device_handle_;
  }

  viennamini::config & config()
  {
    return *config_handle_;
  }

  virtual void run(segment_values& current_contact_potentials, 
                   segment_values& current_contact_workfunctions, 
                   std::size_t step_id) = 0;

  void write(std::string const& filename, std::size_t step_id)
  {
    if(device().is_line1d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device().get_problem_description_line_1d(step_id).quantities(), 
        filename, 
        device().get_segmesh_line_1d().mesh, 
        device().get_segmesh_line_1d().segmentation);
    }
    else
    if(device().is_triangular2d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device().get_problem_description_triangular_2d(step_id).quantities(), 
        filename, 
        device().get_segmesh_triangular_2d().mesh, 
        device().get_segmesh_triangular_2d().segmentation);
    }
    else 
    if(device().is_tetrahedral3d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        device().get_problem_description_tetrahedral_3d(step_id).quantities(), 
        filename, 
        device().get_segmesh_tetrahedral_3d().mesh, 
        device().get_segmesh_tetrahedral_3d().segmentation);
    }
    else throw device_not_supported_exception("at: problem::write()"); 
  }

  viennamini::csv& csv() { return csv_; }

  std::ostream& stream()
  {
    return stream_;
  }

private:
  viennamini::device_handle            device_handle_;
  viennamini::config_handle            config_handle_;
  viennamini::csv                      csv_;
  std::ostream                       & stream_;
};

} // viennamini

#endif





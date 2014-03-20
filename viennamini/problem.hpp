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

   license:    see file LICENSE in the ViennaMini base directory
======================================================================= */

#ifndef NDEBUG
  #define NDEBUG
#endif

// ViennaMath includes:
#include "viennamath/expression.hpp"

#include "viennamini/forwards.h"
#include "viennamini/simulator.hpp"
#include "viennamini/physics.hpp"
#include "viennamini/data_table.hpp"
#include "viennamini/utils/is_zero.hpp"
#include "viennamini/stepper.hpp"

#include <boost/lexical_cast.hpp>


#define VIENNAMINI_PROBLEM(classname) \
public: \
\
  classname(std::ostream& stream) : viennamini::problem(stream) {} \
\
  void run(viennamini::stepper& stepper) \
  {\
    if(device().is_line1d()) \
    {\
      this->run_impl(simulator().device().get_segmesh_line_1d(), stepper); \
    }\
    else \
    if(device().is_triangular2d()) \
    {\
      this->run_impl(simulator().device().get_segmesh_triangular_2d(), stepper); \
    }\
    else \
    if(device().is_tetrahedral3d()) \
    {\
      this->run_impl(simulator().device().get_segmesh_tetrahedral_3d(), stepper); \
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

/** @brief Exception for the case that the solution did not converge */
class solution_not_converged_error : public std::runtime_error {
public:
  solution_not_converged_error(std::string const & str) : std::runtime_error(str) {}
};

/** @brief Exception for the case that a required quantity does not contain values */
class required_quantity_is_zero_exception : public std::runtime_error {
public:
  required_quantity_is_zero_exception(std::string const & str) : std::runtime_error(str) {}
};

class required_quantity_missing : public std::runtime_error {
public:
  required_quantity_missing(std::string const & str) : std::runtime_error(str) {}
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

  problem(std::ostream& stream = std::cout);

  virtual ~problem();

  void set_device_handle(viennamini::device_handle new_device);
  void set_config_handle(viennamini::config_handle new_config);

  viennamini::device & device();
  viennamini::config & config();

  virtual void run(viennamini::stepper& stepper) = 0;

  void write(std::string const& filename, std::size_t step_id);

  viennamini::data_table& data_table();

  std::ostream& stream();

private:
  viennamini::data_table               data_table_;
  viennamini::device_handle            device_handle_;
  viennamini::config_handle            config_handle_;
  std::ostream                       & stream_;
};

} // viennamini

#endif


#ifndef VIENNAMINI_CONFIGS_LINEARSOLVER_HPP
#define VIENNAMINI_CONFIGS_LINEARSOLVER_HPP

/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */

/** @file viennamini/configs/linear_solver.hpp
    @brief Configuration class for linear solver backends
*/

#include "viennamini/forwards.h"

namespace viennamini {
namespace config {

/** @brief Exception class for the linear solver configuration 
*/
class linear_solver_exception : public std::runtime_error {
public:
  /** @brief The constructor expects the exception message
  *
  * @param str The exception message
  */
  linear_solver_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief Linear solver configuration class. Provides ready-to-use default parameters
*          as well as the ability to personalize the parameters of linear solvers.
*/
struct linear_solver
{
public:
  /** @brief The constructor sets the default values
  */
  linear_solver();

  /** @brief Access method returns a reference to the iterations parameter. 
  *          Can be used to read and overwrite the number of linear solver iterations
  *
  * @return Reference to the number of linear solver iterations
  */
  std::size_t&          iterations();

  /** @brief Access method returns a reference to the break tolerance parameter. 
  *          Can be used to read and overwrite the tolerance up until which the 
  *          linear solver will stop the iterative process.
  *
  * @return Reference to the break tolerance value
  */
  viennamini::numeric&  breaktol();

private:
  std::size_t           iterations_;
  viennamini::numeric   breaktol_;
};

} // config
} // viennamini


#endif


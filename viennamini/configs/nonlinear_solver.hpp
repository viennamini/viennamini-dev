#ifndef VIENNAMINI_CONFIGS_NONLINEARSOLVER_HPP
#define VIENNAMINI_CONFIGS_NONLINEARSOLVER_HPP

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

/** @file viennamini/configs/nonlinear_solver.hpp
    @brief Configuration class for nonlinear solver backends
*/

#include "viennamini/forwards.h"

namespace viennamini {
namespace config {

/** @brief Exception class for the nonlinear solver configuration 
*/
class nonlinear_solver_exception : public std::runtime_error {
public:
  /** @brief The constructor expects the exception message
  *
  * @param str The exception message
  */
  nonlinear_solver_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief Nonlinear solver configuration class. Provides ready-to-use default parameters
*          as well as the ability to personalize the parameters of nonlinear solvers.
*/
struct nonlinear_solver
{
public:
  /** @brief The constructor sets the default values
  */
  nonlinear_solver();

  /** @brief Access method returns a reference to the iterations parameter. 
  *          Can be used to read and overwrite the number of nonlinear solver iterations
  *
  * @return Reference to the number of nonlinear solver iterations
  */
  std::size_t&           iterations();

  /** @brief Access method returns a reference to the break tolerance parameter. 
  *          Can be used to read and overwrite the tolerance up until which the 
  *          nonlinear solver will stop the iterative process.
  *
  * @return Reference to the break tolerance value
  */
  viennamini::numeric&   breaktol();

  /** @brief Access method returns a reference to the damping factor. 
  *          Can be used to read and overwrite the damping value which is used 
  *          by the nonlinear solver to improve the convergence behavior.
  *
  * @return Reference to the damping factor
  */
  viennamini::numeric&   damping();

private:
  std::size_t                   iterations_;
  viennamini::numeric           breaktol_;
  viennamini::numeric           damping_;
};

} // config
} // viennamini


#endif


#ifndef VIENNAMINI_CONFIGS_MODEL_HPP
#define VIENNAMINI_CONFIGS_MODEL_HPP

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

/** @file viennamini/configs/model.hpp
    @brief Configuration class for the physical model to be simulated
*/

#include "viennamini/forwards.h"


namespace viennamini {
namespace config {

/** @brief Exception class for the model configuration 
*/
class model_exception : public std::runtime_error {
public:
  /** @brief The constructor expects the exception message
  *
  * @param str The exception message
  */
  model_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief Physical model class. Allows to set and to configure the physical model 
*          to be simulated by setting a 'PDE set' as well as a 'discretization scheme'.
*/
struct model
{
public:
  model();

  viennamini::discret::discret_ids & discretization_id();

  viennamini::pdeset::pdeset_ids   & pdeset_id();

//  void set_pdeset (viennamini::pdeset::pdeset_ids   pdeset_id );

//  void set_discretization(viennamini::discret::discret_ids discret_id);

//  viennamini::discret::discret_ids& get_discret_id();

//  viennamini::pde_set&              get_pde_set();

private:
  viennamini::discret::discret_ids  discret_id_;
  viennamini::pdeset::pdeset_ids    pdeset_id_;
//  viennamini::pde_set_handle        pde_set_handle_;
};

} // config
} // viennamini


#endif


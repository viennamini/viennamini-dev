#ifndef VIENNAMINI_CONFIGS_MODEL_HPP
#define VIENNAMINI_CONFIGS_MODEL_HPP

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

#include "viennamini/forwards.h"
#include "viennamini/pde_sets/laplace.hpp"
#include "viennamini/pde_sets/drift_diffusion.hpp"

namespace viennamini {
namespace config {

class model_exception : public std::runtime_error {
public:
  model_exception(std::string const & str) : std::runtime_error(str) {}
};

struct model
{
public:
  model();

  void set_pdeset (viennamini::pdeset::pdeset_ids   pdeset_id );

  void set_discretization(viennamini::discret::discret_ids discret_id);

  viennamini::discret::discret_ids get_discret_id();

  viennamini::pde_set& get_pde_set();

private:
  viennamini::discret::discret_ids  discret_id_;

  viennamini::pde_set_handle        pde_set_handle_;
};

} // config
} // viennamini


#endif


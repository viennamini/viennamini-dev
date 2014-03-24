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
  model() : 
    discret_id_ (viennamini::discret::fvm)  {}

  void use_pdeset (viennamini::pdeset::pdeset_ids   pdeset_id ) 
  { 
    if(pdeset_id == viennamini::pdeset::laplace)
    {
      pde_set_handle_ = viennamini::pde_set_handle(new viennamini::laplace());
    }
    else if(pdeset_id == viennamini::pdeset::drift_diffusion)
    {
      pde_set_handle_ = viennamini::pde_set_handle(new viennamini::drift_diffusion());
    }
    else throw model_exception("PDE Set type is not supported!");
  }

  void use_discretization(viennamini::discret::discret_ids discret_id) { discret_id_ = discret_id; }

  viennamini::discret::discret_ids get_discret_id() { return discret_id_; }

  viennamini::pde_set& pde_set() { return *pde_set_handle_; }

private:
  viennamini::discret::discret_ids  discret_id_;

  viennamini::pde_set_handle        pde_set_handle_;
};

} // config
} // viennamini


#endif


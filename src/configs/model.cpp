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

#include "viennamini/configs/model.hpp"

namespace viennamini {
namespace config {

model::model() : discret_id_ (viennamini::discret::fvm)
{

}

void model::set_pdeset (viennamini::pdeset::pdeset_ids   pdeset_id )
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

void model::set_discretization(viennamini::discret::discret_ids discret_id)
{
  discret_id_ = discret_id;
}

viennamini::discret::discret_ids model::get_discret_id()
{
  return discret_id_;
}

viennamini::pde_set& model::get_pde_set()
{
  return *pde_set_handle_;
}


} // config
} // viennamini

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


#include "viennamini/simulator.hpp"
#include "viennamini/prepare_poisson_drift_diffusion_np.hpp"

namespace viennamini
{

simulator::simulator() : device_(storage_), storage_changed_(true), device_changed_(true), 
  config_changed_(true), matlib_changed_(true)
{
}

viennamini::data_storage  const& simulator::storage() const
{
  return storage_;
}

viennamini::data_storage       & simulator::storage()
{
  storage_changed_ = true;
  return storage_;
}

viennamini::device const& simulator::device() const
{
  return device_;
}

viennamini::device      & simulator::device()
{
  device_changed_ = true; 
  return device_;
}

viennamini::config const& simulator::config() const
{
  return config_;
}

viennamini::config      & simulator::config()
{
  config_changed_ = true; 
  return config_;
}

viennamini::material_library const& simulator::material_library() const
{
  return matlib_;
}

viennamini::material_library      & simulator::material_library()
{
  matlib_changed_ = true; 
  return matlib_;
}

void simulator::run()
{
  if(device_changed_)
  {
    std::cout << "updating device .." << std::endl;
    device_.update();
    config_changed_ = true;
  }

  if(config_changed_)
  {
    std::cout << "updating configuration .." << std::endl;
    if(config_.problem() == viennamini::poisson_drift_diffusion_np)
    {
      viennamini::prepare_poisson_drift_diffusion_np(device_);
    }
  }
}

} // viennamini



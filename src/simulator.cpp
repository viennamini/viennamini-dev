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
#include "viennamini/problems/poisson_dd-np.hpp"
#include "viennamini/problems/laplace.hpp"

namespace viennamini
{

simulator::simulator() : 
  device_          (new viennamini::device()),
  config_          (new viennamini::config()), 
  problem_         (NULL),
  device_changed_  (true),
  config_changed_  (true)
{
}

simulator::~simulator()
{
  if(problem_)                       delete problem_;
}

viennamini::device const& simulator::device() const
{
  return *device_;
}

viennamini::device      & simulator::device()
{
  device_changed_ = true; 
  return *device_;
}

void simulator::set_device(viennamini::device_handle& new_device)
{
  device_.reset();
  device_ = new_device;
}

viennamini::config const& simulator::config() const
{
  return *config_;
}

viennamini::config      & simulator::config()
{
  config_changed_ = true; 
  return *config_;
}

void simulator::set_config(viennamini::config_handle& new_config)
{
  config_.reset();
  config_ = new_config;
}

void simulator::run()
{
  if(device_changed_)
  {
  #ifdef VIENNAMINI_VERBOSE
    std::cout << "updating device .." << std::endl;
  #endif
    device().update();
  }

  if(config_changed_ || device_changed_)
  {
  #ifdef VIENNAMINI_VERBOSE  
    std::cout << "updating configuration .." << std::endl;
  #endif
    if(config().problem() == viennamini::id::poisson_drift_diffusion_np())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_poisson_dd_np(device(), config());
      problem_->run();
    }
    else
    if(config().problem() == viennamini::id::laplace())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_laplace(device(), config());
      problem_->run();
    }
    else
    {
      std::cout << "Error: no problem defined in the configuration .." << std::endl;
    }
  }
}

void simulator::write(std::string const filename)
{
  if(problem_) problem_->write(filename);
}

} // viennamini



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
#include "viennamini/problem_poisson_dd-np.hpp"

namespace viennamini
{

simulator::simulator() : device_(storage_), problem_(NULL), 
  storage_changed_(true), device_changed_(true), config_changed_(true), matlib_changed_(true) 
{
}

simulator::~simulator()
{
  if(problem_) delete problem_;
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
  }

  if(config_changed_ || device_changed_ || matlib_changed_)
  {
    std::cout << "updating configuration .." << std::endl;
    if(config_.problem() == viennamini::id::poisson_drift_diffusion_np())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_poisson_dd_np(device_, config_, matlib_);
      problem_->run();
    }
  }
}

void simulator::write(std::string const filename)
{
  if(problem_) problem_->write(filename);
}

} // viennamini



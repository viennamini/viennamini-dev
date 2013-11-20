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
#include "viennamini/problem_laplace.hpp"

namespace viennamini
{

simulator::simulator() : 
  storage_         (new viennamini::data_storage),
  device_          (new viennamini::device(*storage_)),
  config_          (new viennamini::config), 
  matlib_          (new viennamini::material_library),
  problem_         (NULL),
  storage_changed_ (true),
  device_changed_  (true),
  config_changed_  (true),
  matlib_changed_  (true),
  external_storage_(false),
  external_device_ (false),
  external_config_ (false),
  external_matlib_ (false)
{
}

simulator::~simulator()
{
  if(storage_ && !external_storage_) delete storage_;
  if(device_  && !external_device_)  delete device_;
  if(config_  && !external_config_)  delete config_;
  if(matlib_  && !external_matlib_)  delete matlib_;
  if(problem_)                       delete problem_;
}

viennamini::data_storage  const& simulator::storage() const
{
  return *storage_;
}

viennamini::data_storage       & simulator::storage()
{
  storage_changed_ = true;
  return *storage_;
}

void simulator::set_storage(viennamini::data_storage& new_storage)
{
  if(storage_) delete storage_;
  storage_ = &new_storage;
  external_storage_ = true;
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

void simulator::set_device(viennamini::device& new_device)
{
  if(device_)  delete device_;
  device_ = &new_device;
  external_device_ = true;
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

void simulator::set_config(viennamini::config& new_config)
{
  if(config_)  delete config_;
  config_ = &new_config;
  external_config_ = true;
}

viennamini::material_library const& simulator::material_library() const
{
  return *matlib_;
}

viennamini::material_library      & simulator::material_library()
{
  matlib_changed_ = true; 
  return *matlib_;
}

void simulator::set_material_library(viennamini::material_library& new_material_library)
{
  if(matlib_)  delete matlib_;
  matlib_ = &new_material_library;
  external_matlib_ = true;
}

void simulator::run()
{
  if(device_changed_)
  {
    std::cout << "updating device .." << std::endl;
    device().update();
  }

  if(config_changed_ || device_changed_ || matlib_changed_)
  {
    std::cout << "updating configuration .." << std::endl;
    if(config().problem() == viennamini::id::poisson_drift_diffusion_np())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_poisson_dd_np(device(), config(), material_library());
      problem_->run();
    }
    else
    if(config().problem() == viennamini::id::laplace())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_laplace(device(), config(), material_library());
      problem_->run();
    }
  }
}

void simulator::write(std::string const filename)
{
  if(problem_) problem_->write(filename);
}

} // viennamini



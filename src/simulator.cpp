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
#include "viennamini/utils/convert.hpp"
#include "viennamini/problems/poisson_dd-np.hpp"
#include "viennamini/problems/laplace.hpp"

namespace viennamini
{

simulator::simulator(std::ostream& stream) : 
  device_handle_   (new viennamini::device(stream)),
  config_handle_   (new viennamini::config(stream)), 
  stepper_         (device_handle_),
  problem_         (NULL),
  stream_          (stream),
  output_file_prefix_("output"),
  device_changed_  (true),
  config_changed_  (true),
  manual_problem_  (false)
{
}

simulator::~simulator()
{
  if(problem_)                       delete problem_;
}

viennamini::device const& simulator::device() const
{
  return *device_handle_;
}

viennamini::device      & simulator::device()
{
  device_changed_ = true; 
  return *device_handle_;
}

void simulator::set_device(viennamini::device_handle& new_device)
{
  device_handle_.reset();
  device_handle_ = new_device;
}

viennamini::device_handle& simulator::device_handle()
{
  return device_handle_;
}

viennamini::config const& simulator::config() const
{
  return *config_handle_;
}

viennamini::config      & simulator::config()
{
  config_changed_ = true; 
  return *config_handle_;
}

void simulator::set_config(viennamini::config_handle& new_config)
{
  config_handle_.reset();
  config_handle_ = new_config;
}

viennamini::config_handle& simulator::config_handle()
{
  return config_handle_;
}

void simulator::set_problem(viennamini::problem* active_problem)
{
  if(!active_problem) throw problem_not_available_exception("");
  problem_ = active_problem;
  manual_problem_ = true;
}

void simulator::run()
{
  if(device_changed_)
  {
  #ifdef VIENNAMINI_VERBOSE
    stream() << "[Simulator] device has changed, updating .."  << std::endl;
  #endif
    device().update();
  }

  if(config_changed_ || device_changed_)
  {
    if(stepper().empty())
    {
      this->run_impl(0);
      
      if(config().write_result_files())
        problem_->write(output_file_prefix_);
    }
    else 
    {
      // resize the device's problem description container?!
    
      while(stepper().apply_next())
      {
        this->run_impl(stepper().get_current_step_id());
        
        if(config().write_result_files())
          problem_->write(output_file_prefix_+"_"+stepper().get_current_step_setup_string());
      }
    }
  }
}

void simulator::run_impl(std::size_t step_id)
{
  if(manual_problem_)
  {
  #ifdef VIENNAMINI_VERBOSE
    stream() << "[Simulator] processing manual problem .."  << std::endl;
  #endif
    problem_->set(this->device_handle(), this->config_handle());
    problem_->run();
  }
  else
  {
  #ifdef VIENNAMINI_VERBOSE
    stream() << "[Simulator] processing problem \"" << config().problem() << "\""  << std::endl;
  #endif
    if(config().problem() == viennamini::id::poisson_drift_diffusion_np())
    {
      if(problem_) delete problem_;
      problem_ = new viennamini::problem_poisson_dd_np(this->stream());
      problem_->set(this->device_handle(), this->config_handle());
      problem_->run();
    }
    else
    if(config().problem() == viennamini::id::laplace())
    {

      if(problem_) delete problem_;
      problem_ = new viennamini::problem_laplace(this->stream());
      problem_->set(this->device_handle(), this->config_handle());
      problem_->run();
    }
    else throw undefined_problem_exception("Problem \""+config().problem()+"\" not recognized");
  }
}

viennamini::stepper& simulator::stepper()
{
  return stepper_;
}

std::ostream& simulator::stream()
{
  return stream_;
}

void simulator::set_output_filename_prefix(std::string const prefix)
{
  output_file_prefix_ = prefix;
}

} // viennamini



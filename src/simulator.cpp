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
#include "viennamini/device_template.hpp"
#include "viennamini/utils/convert.hpp"
#include "viennamini/problems/poisson_dd-np.hpp"
#include "viennamini/problems/laplace.hpp"

namespace viennamini
{

simulator::simulator(std::ostream& stream) :
  stepper_         (current_contact_potentials_),
  device_generator_(NULL),
  problem_id_      (""),
  device_handle_   (new viennamini::device(stream)),
  config_handle_   (new viennamini::config(stream)),
  problem_         (NULL),
  stream_          (stream),
  output_file_prefix_("output"),
  device_changed_  (true),
  config_changed_  (true),
  manual_problem_  (false)
{
}

simulator::simulator(device_template* device_generator, std::ostream& stream) :
  stepper_         (current_contact_potentials_),
  device_generator_(device_generator),
  problem_id_      (device_generator->problem_id()),
  device_handle_   (device_generator->device_handle()),
  config_handle_   (device_generator->config_handle()),
  problem_         (NULL),
  stream_          (device_generator->stream()),
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

viennamini::config_handle& simulator::config_handle()
{
  return config_handle_;
}

std::string& simulator::problem_id()
{
  return problem_id_;
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
      if(manual_problem_)
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "[Simulator] processing manual problem .."  << std::endl;
      #endif
        problem_->set(this->device_handle(), this->config_handle());
        problem_->run(current_contact_potentials_, current_contact_workfunctions_, 0);
      }
      else
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "[Simulator] processing problem \"" << config().problem() << "\""  << std::endl;
      #endif
        if(problem_id() == viennamini::id::poisson_drift_diffusion_np())
        {
          if(problem_) delete problem_;
          problem_ = new viennamini::problem_poisson_dd_np(this->stream());
          problem_->set(this->device_handle(), this->config_handle());
          problem_->run(current_contact_potentials_, current_contact_workfunctions_, 0);
        }
        else
        if(problem_id() == viennamini::id::laplace())
        {

          if(problem_) delete problem_;
          problem_ = new viennamini::problem_laplace(this->stream());
          problem_->set(this->device_handle(), this->config_handle());
          problem_->run(current_contact_potentials_, current_contact_workfunctions_, 0);
        }
        else throw undefined_problem_exception("Problem \""+problem_id()+"\" not recognized");
      }

      if(config().write_result_files())
        problem_->write(output_file_prefix_, 0);
    }
    else // perform a sequence of simulations ..
    {
      // make sure, that the problem description is ready to hold
      // the simulation data for all upcoming simulations
      this->resize_problem_description_set();

      if(manual_problem_)
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "[Simulator] processing manual problem .."  << std::endl;
      #endif
        problem_->set(this->device_handle(), this->config_handle());

        this->execute_loop();
      }
      else
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "[Simulator] processing problem \"" << problem_id() << "\""  << std::endl;
      #endif
        if(problem_id() == viennamini::id::poisson_drift_diffusion_np())
        {
          if(problem_) delete problem_;
          problem_ = new viennamini::problem_poisson_dd_np(this->stream());
          problem_->set(this->device_handle(), this->config_handle());
          this->execute_loop();
        }
        else
        if(problem_id() == viennamini::id::laplace())
        {

          if(problem_) delete problem_;
          problem_ = new viennamini::problem_laplace(this->stream());
          problem_->set(this->device_handle(), this->config_handle());
          this->execute_loop();
        }
        else throw undefined_problem_exception("Problem \""+problem_id()+"\" not recognized");
      }
    }
  }
}

void simulator::execute_loop()
{
  while(stepper().apply_next())
  {
    // for the output, use 1-based indices, helping the user to keep track of the iteration numbers
    stream() << "Executing simulation " << stepper().get_current_step_id()+1 << " of " << stepper().size();
    problem_->run(current_contact_potentials_, current_contact_workfunctions_, stepper().get_current_step_id());
    if(config().write_result_files())
    {
      stream() << "  --> " << output_file_prefix_+"_"+this->encode_current_boundary_setup() << std::endl;
      problem_->write(output_file_prefix_+"_"+this->encode_current_boundary_setup(), stepper().get_current_step_id());
    }
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

viennamini::numeric& simulator::current_contact_potential   (std::size_t segment_index)
{
  return current_contact_potentials_[segment_index];
}

viennamini::numeric& simulator::current_contact_workfunction(std::size_t segment_index)
{
  return current_contact_workfunctions_[segment_index];
}

viennamini::device_template& simulator::device_generator()
{
  return *device_generator_;
}

viennamini::csv& simulator::csv()
{
  return problem_->csv();
}

std::string simulator::encode_current_boundary_setup()
{
  std::string result;
  for(stepper::step_setup_type::iterator iter = stepper().get_current_step_setup().begin();
      iter != stepper().get_current_step_setup().end(); iter++)
  {
    result += device().get_name(iter->first) + "=" + viennamini::convert<std::string>()(iter->second);
    if((iter+1) != stepper().get_current_step_setup().end()) result += "_";
  }
  return result;
}

void simulator::resize_problem_description_set()
{
  if(device().is_line1d())
  {
    for(std::size_t i = 0; i < stepper_.size()-1; i++) // -1 because there is already one by default
    {
      device().get_problem_description_line_1d_set().push_back( problem_description_line_1d(device().get_segmesh_line_1d().mesh) );
    }
  }
  else
  if(device().is_triangular2d())
  {
    for(std::size_t i = 0; i < stepper_.size()-1; i++) // -1 because there is already one by default
    {
      device().get_problem_description_triangular_2d_set().push_back( problem_description_triangular_2d(device().get_segmesh_triangular_2d().mesh) );
    }
  }
  else
  if(device().is_tetrahedral3d())
  {
    for(std::size_t i = 0; i < stepper_.size()-1; i++) // -1 because there is already one by default
    {
      device().get_problem_description_tetrahedral_3d_set().push_back( problem_description_tetrahedral_3d(device().get_segmesh_tetrahedral_3d().mesh) );
    }
  }
  else throw device_not_supported_exception("at: simulator::resize_problem_description_set()");
}

} // viennamini



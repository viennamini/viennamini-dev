/* =======================================================================
   Copyright (c) 2011-2013, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */


#include "viennamini/simulator.hpp"
#include "viennamini/device_template.hpp"
#include "viennamini/utils/convert.hpp"
//#include "viennamini/problems/poisson_dd-np.hpp"
//#include "viennamini/problems/laplace.hpp"

namespace viennamini
{

simulator::simulator(std::ostream& stream) :
  device_handle_   (new viennamini::device(stream)),
  config_handle_   (new viennamini::configuration(stream)),
  stepper_handle_  (new viennamini::stepper()),
  stream_          (stream),
  device_changed_  (true),
  config_changed_  (true)
{
}

simulator::~simulator()
{
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

void simulator::set_device_handle(viennamini::device_handle other_device_handle)
{
  device_changed_ = true;
  device_handle_ = other_device_handle;
}

viennamini::configuration const& simulator::config() const
{
  return *config_handle_;
}

viennamini::configuration      & simulator::config()
{
  config_changed_ = true;
  return *config_handle_;
}

viennamini::configuration_handle& simulator::config_handle()
{
  return config_handle_;
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

  if(config().model().get_discret_id() == viennamini::discret::fvm)
  {
    discretization_handle_ = discretization_handle(new viennamini::fvm(device_handle_, config_handle_, stepper_handle_, stream_));
    discretization_handle_->run();
  }


  /*


  if(model().get_discret() == viennamini::discret::fvm)
  {
    //viennamini::myfvm(*device_handle_, *config_handle_, stepper_, stream_);
  }
  else throw discretization_not_supported();

  */



//  if(device_changed_)
//  {
//  #ifdef VIENNAMINI_VERBOSE
//    stream() << "[Simulator] device has changed, updating .."  << std::endl;
//  #endif
//    device().update();
//  }

//  if(config_changed_ || device_changed_)
//  {
//    // load the stepper with the contact setup
//    //
//    stepper().clear();
//    for(viennamini::device::IndicesType::iterator siter = device().segment_indices().begin();
//        siter != device().segment_indices().end(); siter++)
//    {
//      if(device().is_contact(*siter))
//      {  
//        // if the contact is neither set to range or single,
//        // make it a single value contact
//        //
//        if( (!is_contact_single(*siter)) && (!is_contact_range(*siter)) )
//          is_contact_single(*siter) = true;

//        // initialize unset contacts with default values
//        //
//        if(is_contact_single(*siter))
//        {
//          if(contact_potentials_.find(*siter) == contact_potentials_.end())
//            contact_potentials_[*siter] = 0.0;
//        }
//        if(contact_workfunctions_.find(*siter) == contact_workfunctions_.end())
//          contact_workfunctions_[*siter] = 0.0;

//        // now, setup the contact stepper by adding the contact values
//        //
//        if(is_contact_single(*siter))
//        {
//          stepper().add(*siter, contact_potential(*siter));
//        }
//        else if(is_contact_range(*siter))
//        {
//          stepper().add(*siter, contact_potential_range_from(*siter),
//                                contact_potential_range_to(*siter),
//                                contact_potential_range_delta(*siter));
//        }
//      }
//    }



//    if(manual_problem_)
//    {
//    #ifdef VIENNAMINI_VERBOSE
//      stream() << "[Simulator] processing manual problem .."  << std::endl;
//    #endif
//      problem_->set_device_handle(device_handle_);
//      problem_->set_config_handle(config_handle_);
//        problem_->run(stepper());
//      //this->execute_loop();
//    }
//    else
//    {
//    #ifdef VIENNAMINI_VERBOSE
//      stream() << "[Simulator] processing problem \"" << problem_id() << "\""  << std::endl;
//    #endif
//      if(problem_id() == viennamini::id::poisson_drift_diffusion_np())
//      {
//        if(problem_) delete problem_;
//        problem_ = new viennamini::problem_poisson_dd_np(this->stream());
//        problem_->set_device_handle(device_handle_);
//        problem_->set_config_handle(config_handle_);
//        problem_->run(stepper());
//        //this->execute_loop();
//      }
//      else
//      if(problem_id() == viennamini::id::laplace())
//      {
//        if(problem_) delete problem_;
//        problem_ = new viennamini::problem_laplace(this->stream());
//        problem_->set_device_handle(device_handle_);
//        problem_->set_config_handle(config_handle_);
//        problem_->run(stepper());
//        //this->execute_loop();
//      }
//      else
//      if(problem_id() == "")
//        throw undefined_problem_exception("Problem has not been defined");
//      else
//        throw undefined_problem_exception("Problem \""+problem_id()+"\" not recognized");
//    }
//  }
}

//void simulator::execute_loop()
//{
//  segment_values current_contact_potentials;
//  while(stepper().apply_next(current_contact_potentials))
//  {
//    // for the output, use 1-based indices, helping the user to keep track of the iteration numbers
//    stream() << "Executing simulation " << stepper().get_current_step_id() << " of " << stepper().size();
//    problem_->run(current_contact_potentials, stepper().get_current_step_id());
//    if(config().write_result_files())
//    {
//      stream() << "  --> " << output_file_prefix_+"_"+this->encode_current_boundary_setup() << std::endl;
//      problem_->write(output_file_prefix_+"_"+this->encode_current_boundary_setup(), stepper().get_current_step_id());
//    }
//  }
//}

//viennamini::stepper& simulator::stepper()
//{
//  return stepper_;
//}

std::ostream& simulator::stream()
{
  return stream_;
}

//void simulator::set_output_filename_prefix(std::string const prefix)
//{
//  output_file_prefix_ = prefix;
//}

//viennamini::numeric& simulator::contact_potential   (std::size_t segment_index)
//{
//  this->is_contact_single(segment_index) = true;
//  this->is_contact_range(segment_index) = false;
//  return contact_potentials_[segment_index];
//}

//viennamini::numeric& simulator::contact_potential_range_from   (std::size_t segment_index)
//{
//  this->is_contact_single(segment_index) = false;
//  this->is_contact_range(segment_index) = true;
//  return contact_potential_range_from_[segment_index];
//}

//viennamini::numeric& simulator::contact_potential_range_to     (std::size_t segment_index)
//{
//  this->is_contact_single(segment_index) = false;
//  this->is_contact_range(segment_index) = true;
//  return contact_potential_range_to_[segment_index];
//}

//viennamini::numeric& simulator::contact_potential_range_delta  (std::size_t segment_index)
//{
//  this->is_contact_single(segment_index) = false;
//  this->is_contact_range(segment_index) = true;
//  return contact_potential_range_delta_[segment_index];
//}

//void simulator::set_contact_potential_range(std::size_t segment_index, viennamini::numeric from, viennamini::numeric to, viennamini::numeric delta)
//{
//  this->contact_potential_range_from(segment_index) = from;
//  this->contact_potential_range_to(segment_index) = to;
//  this->contact_potential_range_delta(segment_index) = delta;
//}

//viennamini::numeric& simulator::contact_workfunction(std::size_t segment_index)
//{
//  return contact_workfunctions_[segment_index];
//}

//bool& simulator::is_contact_single(std::size_t segment_index)
//{
//  return contact_single_flags_[segment_index];
//}

//bool& simulator::is_contact_range(std::size_t segment_index)
//{
//  return contact_range_flags_[segment_index];
//}

//bool& simulator::record_iv(std::size_t segment_index)
//{
//  return record_iv_flags_[segment_index];
//}

//viennamini::device_template& simulator::device_generator()
//{
//  return *device_generator_;
//}

//viennamini::data_table& simulator::data_table()
//{
//  return problem_->data_table();
//}

//std::string simulator::encode_current_boundary_setup()
//{
//  std::string result;
//  for(stepper::step_setup_type::iterator iter = stepper().get_current_step_setup().begin();
//      iter != stepper().get_current_step_setup().end(); iter++)
//  {
//    result += device().get_name(iter->first) + "=" + viennamini::convert<std::string>()(iter->second);
//    if((iter+1) != stepper().get_current_step_setup().end()) result += "_";
//  }
//  return result;
//}

//void simulator::resize_problem_description_set()
//{
//  if(device().is_line1d())
//  {
//    // clean 1-n entries of the problem description set
//    // note that these entries hold previous simulation results
//    // the 0 entry holds the initial values, so we keep this one
//    //
//    device().get_problem_description_line_1d_set().erase(
//      device().get_problem_description_line_1d_set().begin()+1,
//      device().get_problem_description_line_1d_set().end());

//    // now, create new problem descriptions for each simulation to be conducted
//    //
//    for(std::size_t i = 0; i < stepper_.size(); i++)
//    {
//      device().get_problem_description_line_1d_set().push_back( problem_description_line_1d(device().get_segmesh_line_1d().mesh) );
//    }
//  }
//  else
//  if(device().is_triangular2d())
//  {
//    // clean 1-n entries of the problem description set
//    // note that these entries hold previous simulation results
//    // the 0 entry holds the initial values, so we keep this one
//    //
//    device().get_problem_description_triangular_2d_set().erase(
//      device().get_problem_description_triangular_2d_set().begin()+1,
//      device().get_problem_description_triangular_2d_set().end());

//    // now, create new problem descriptions for each simulation to be conducted
//    //
//    for(std::size_t i = 0; i < stepper_.size(); i++) 
//    {
//      device().get_problem_description_triangular_2d_set().push_back( problem_description_triangular_2d(device().get_segmesh_triangular_2d().mesh) );
//    }
//  }
//  else
//  if(device().is_tetrahedral3d())
//  {
//    // clean 1-n entries of the problem description set
//    // note that these entries hold previous simulation results
//    // the 0 entry holds the initial values, so we keep this one
//    //
//    device().get_problem_description_tetrahedral_3d_set().erase(
//      device().get_problem_description_tetrahedral_3d_set().begin()+1,
//      device().get_problem_description_tetrahedral_3d_set().end());

//    // now, create new problem descriptions for each simulation to be conducted
//    //
//    for(std::size_t i = 0; i < stepper_.size(); i++) // -1 because there is already one by default
//    {
//      device().get_problem_description_tetrahedral_3d_set().push_back( problem_description_tetrahedral_3d(device().get_segmesh_tetrahedral_3d().mesh) );
//    }
//  }
//  else throw device_not_supported_exception("at: simulator::resize_problem_description_set()");
//}

} // viennamini



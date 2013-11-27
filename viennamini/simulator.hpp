#ifndef VIENNAMINI_SIMULATOR_HPP
#define VIENNAMINI_SIMULATOR_HPP

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

#ifndef NDEBUG
  #define NDEBUG
#endif

// ViennaMini includes:
#include "viennamini/forwards.h"
#include "viennamini/config.hpp"
#include "viennamini/device.hpp"
#include "viennamini/problem.hpp"
#include "viennamini/stepper.hpp"
#include "viennamini/utils/convert.hpp"

namespace viennamini
{
  /** @brief Exception for the case that the input material library xml file is not supported */
  class undefined_problem_exception : public std::runtime_error {
  public:
    undefined_problem_exception(std::string const & str) : std::runtime_error(str) {}
  };

  /** @brief Exception for the case that a manually provided problem is not available, i.e., it's not instantiated */
  class problem_not_available_exception : public std::runtime_error {
  public:
    problem_not_available_exception(std::string const & str) : std::runtime_error(str) {}
  };

  class simulator
  {
  public:
  
    simulator(std::ostream& stream = std::cout);
    ~simulator();

    void                                 run();

    viennamini::device            const& device() const;
    viennamini::device                 & device();
    void                                 set_device(viennamini::device_handle& new_device);
    viennamini::device_handle          & device_handle();

    viennamini::config            const& config() const;
    viennamini::config                 & config();
    void                                 set_config(viennamini::config_handle& new_config);
    viennamini::config_handle          & config_handle();

    viennamini::stepper                & stepper();
    std::ostream                       & stream();

    viennamini::numeric                & current_contact_potential   (std::size_t segment_index);
    viennamini::numeric                & current_contact_workfunction(std::size_t segment_index);

    void                                 set_problem(viennamini::problem* active_problem);
    void                                 set_output_filename_prefix(std::string const prefix);
    
    viennamini::csv                    & csv();
  
  private:

    void                                 execute_loop();
    std::string                          encode_current_boundary_setup(); 
    void                                 resize_problem_description_set();

    viennamini::device_handle            device_handle_;
    viennamini::config_handle            config_handle_;
    viennamini::stepper                  stepper_;
    viennamini::problem                * problem_;
    std::ostream                       & stream_;
    std::string                          output_file_prefix_;
    
    segment_values                       current_contact_potentials_;
    segment_values                       current_contact_workfunctions_;
    
    bool device_changed_;
    bool config_changed_;
    bool manual_problem_;
  };
}

#endif


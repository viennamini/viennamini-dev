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

   license:    see file LICENSE in the base directory
======================================================================= */

#ifndef NDEBUG
  #define NDEBUG
#endif

// ViennaMini includes:
#include "viennamini/forwards.h"
#include "viennamini/configuration.hpp"
#include "viennamini/device.hpp"
//#include "viennamini/problem.hpp"
#include "viennamini/stepper.hpp"
#include "viennamini/data_table.hpp"
#include "viennamini/utils/convert.hpp"
#include "viennamini/discretization.hpp"
#include "viennamini/discretizations/fvm.hpp"

namespace viennamini
{
//  /** @brief Generic Exception */
//  class undefined_problem_exception : public std::runtime_error {
//  public:
//    undefined_problem_exception(std::string const & str) : std::runtime_error(str) {}
//  };

//  /** @brief Exception for the case that a manually provided problem is not available, i.e., it's not instantiated */
//  class problem_not_available_exception : public std::runtime_error {
//  public:
//    problem_not_available_exception(std::string const & str) : std::runtime_error(str) {}
//  };

  class simulator_exception : public std::runtime_error {
  public:
    simulator_exception(std::string const & str) : std::runtime_error(str) {}
  };

  class simulator
  {
  public:

    simulator(std::ostream& stream = std::cout);
//    simulator(device_template* device_generator, std::ostream& stream = std::cout);
    ~simulator();

    void                                 run();

    viennamini::device            const& device() const;
    viennamini::device                 & device();
    viennamini::device_handle          & device_handle();
    void                                 set_device_handle(viennamini::device_handle device_handle);

    viennamini::configuration            const& config() const;
    viennamini::configuration                 & config();
    viennamini::configuration_handle          & config_handle();

    viennamini::stepper                & stepper();
    std::ostream                       & stream();

//    std::string                        & problem_id();
//    void                                 set_problem(viennamini::problem* active_problem);
//    void                                 set_output_filename_prefix(std::string const prefix);

    viennamini::data_table             & data_table();

    device_template                    & device_generator();

//    viennamini::numeric                & contact_workfunction           (std::size_t segment_index);
//    viennamini::numeric                & contact_potential              (std::size_t segment_index);
//    viennamini::numeric                & contact_potential_range_from   (std::size_t segment_index);
//    viennamini::numeric                & contact_potential_range_to     (std::size_t segment_index);
//    viennamini::numeric                & contact_potential_range_delta  (std::size_t segment_index);

//    void set_contact_potential_range  (std::size_t segment_index, viennamini::numeric from, viennamini::numeric to, viennamini::numeric delta);

//    bool                               & is_contact_single   (std::size_t segment_index);
//    bool                               & is_contact_range    (std::size_t segment_index);
//    bool                               & record_iv           (std::size_t segment_index);

  private:

//    std::string                          encode_current_boundary_setup();

    viennamini::device_handle            device_handle_;
    viennamini::configuration_handle     config_handle_;
    viennamini::stepper_handle           stepper_handle_;
    viennamini::discretization_handle    discretization_handle_;
    std::ostream                       & stream_;


//    std::string                          output_file_prefix_;
//    device_template                    * device_generator_;
//    segment_values                       contact_potentials_;
//    segment_values                       contact_workfunctions_;
//    segment_values                       contact_potential_range_from_;
//    segment_values                       contact_potential_range_to_;
//    segment_values                       contact_potential_range_delta_;

//    std::map<std::size_t, bool>          contact_single_flags_;
//    std::map<std::size_t, bool>          contact_range_flags_;
//    std::map<std::size_t, bool>          record_iv_flags_;

    bool device_changed_;
    bool config_changed_;
  };
}

#endif


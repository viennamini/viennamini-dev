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
  class simulator_exception : public std::runtime_error {
  public:
    simulator_exception(std::string const & str) : std::runtime_error(str) {}
  };

  class simulator
  {
  public:

    simulator(std::ostream& stream = std::cout);
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

    viennamini::data_table             & data_table();

    device_template                    & device_generator();

  private:

    viennamini::device_handle            device_handle_;
    viennamini::configuration_handle     config_handle_;
    viennamini::stepper_handle           stepper_handle_;
    viennamini::discretization_handle    discretization_handle_;
    std::ostream                       & stream_;

    bool device_changed_;
    bool config_changed_;
  };
}

#endif

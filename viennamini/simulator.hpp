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
#include "viennamini/material_library.hpp"
#include "viennamini/problem.hpp"

namespace viennamini
{
  class simulator
  {
  public:
    simulator();
    ~simulator();

    void                                 run();

    viennamini::device            const& device() const;
    viennamini::device                 & device();
    void                                 set_device(viennamini::device_handle& new_device);

    viennamini::config            const& config() const;
    viennamini::config                 & config();
    void                                 set_config(viennamini::config_handle& new_config);

    viennamini::material_library  const& material_library() const;
    viennamini::material_library       & material_library();
    void                                 set_material_library(viennamini::material_library_handle& new_material_library);

    void                                 write(std::string const filename);
  
  private:

    viennamini::device_handle            device_;
    viennamini::config_handle            config_;
    viennamini::material_library_handle  matlib_;

    viennamini::problem           * problem_;
    
    bool device_changed_;
    bool config_changed_;
    bool matlib_changed_;

  };
}

#endif


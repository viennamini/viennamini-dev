#ifndef VIENNAMINI_MATERIALLIBRARY_HPP
#define VIENNAMINI_MATERIALLIBRARY_HPP

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

// ViennaMaterials includes
#include "viennamaterials/pugixml.hpp"

namespace viennamini {

  class material_library
  {
  public:
    material_library();
    ~material_library();
    void read(std::string const& filename);
    void reset();
    viennamaterials::library* operator()();
    bool is_empty();

  private:
    viennamaterials::library* lib_;
  };
} // viennamini

#endif 


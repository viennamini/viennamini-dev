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


#include "viennamini/material_library.hpp"

#include "viennamaterials/generator.hpp"

namespace viennamini {

material_library::material_library()
{
  lib_ = NULL;
}

material_library::~material_library()
{
  if(lib_) delete lib_;
}

void material_library::read(std::string const& filename)
{
  // if a new file should be imported although another one has 
  // already been imported, reset the database 
  if(lib_) this->reset();
  // create the library by guessing the type from the file ending
  lib_ = viennamaterials::generator(filename);
}

void material_library::reset()
{
  if(lib_) delete lib_;
  lib_ = NULL;
}

} // viennamini


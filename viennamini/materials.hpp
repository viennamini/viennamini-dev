#ifndef VIENNAMINI_MATERIALS_HPP
#define VIENNAMINI_MATERIALS_HPP

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

#include "viennamaterials/forwards.h"
#include "viennamaterials/make_query.hpp"
#include "viennamaterials/base_accessor.hpp"
#include "viennamaterials/utils/file_extension.hpp"
#include "viennamaterials/utils/convert.hpp"



namespace viennamini
{
struct material_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/materials/material[id=\"%\"]";
  }
};

struct parameter_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/parameter[name=\"%\"]";
  }
};

struct data_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/%/text()";
  }
};

}

#endif


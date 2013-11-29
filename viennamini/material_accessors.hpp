#ifndef VIENNAMINI_MATERIALACCESSORS_HPP
#define VIENNAMINI_MATERIALACCESSORS_HPP

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
#include "viennamaterials/base_accessor.hpp"



namespace viennamini
{
struct xpath_material_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/materials/material[id=\"%\"]";
  }
};

struct xpath_model_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/model[id=\"%\"]";
  }
};

struct xpath_parameter_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/parameter[name=\"%\"]";
  }
};

struct xpath_data_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/%/text()";
  }
};

struct xpath_material_category_accessor : public viennamaterials::base_accessor
{
  result_type operator()()
  {
    return "/materials/material[category=\"%\"]";
  }
};

}

#endif


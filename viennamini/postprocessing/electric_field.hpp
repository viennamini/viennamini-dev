#ifndef VIENNAMINI_POSTPROCESSING_ELECTRICFIELD_HPP
#define VIENNAMINI_POSTPROCESSING_ELECTRICFIELD_HPP

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


#include "viennamini/forwards.h"

namespace viennamini {

namespace postproc {

template<typename QuantityT>
struct electric_field
{
  typedef viennamini::numeric numeric_type;
  typedef numeric_type        result_type;

  electric_field(QuantityT& potential) :
    potential_(potential) {}

  template<typename CellT>
  result_type operator()(CellT const& cell)
  {
    
  }

private:
  QuantityT& potential_;
};

} // postproc
} // viennamini

#endif


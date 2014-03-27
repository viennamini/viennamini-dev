#ifndef VIENNAMINI_INITIALGUESS_QUANTITY_HPP
#define VIENNAMINI_INITIALGUESS_QUANTITY_HPP

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

#include <vector>

#include "viennamini/forwards.h"
#include "viennamini/initial_guess.hpp"

namespace viennamini {
namespace init {

class quantity : public initial_guess
{
public:
  quantity(std::string const& quantity_name) : initial_guess(), quantity_name_(quantity_name)
  {
  }

  ~quantity() {}

  result_type operator()(std::size_t cell_index)
  {
    return get_device().get_quantity(quantity_name_, get_segment_index(), cell_index);
  }

private:
  std::string quantity_name_;
};

} // init
} // viennamini


#endif


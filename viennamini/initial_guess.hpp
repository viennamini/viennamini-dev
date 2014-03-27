#ifndef VIENNAMINI_INITIALGUESS_HPP
#define VIENNAMINI_INITIALGUESS_HPP

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
#include "viennamini/device.hpp"

namespace viennamini {
namespace init {

class initial_guess
{
public:
  typedef viennamini::numeric   result_type;

  initial_guess() {}
  virtual ~initial_guess() {}

  virtual result_type operator()(std::size_t cell_index) = 0;

  void set_device(viennamini::device* device)       { device_ = device; }
  void set_segment_index(std::size_t segment_index) { segment_index_ = segment_index; }

  viennamini::device& get_device()        { return *device_; }
  std::size_t&        get_segment_index() { return segment_index_; }

private:
  viennamini::device*       device_;
  std::size_t               segment_index_;
};

} // init
} // viennamini


#endif


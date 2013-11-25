#ifndef VIENNAMINI_STEPPER_HPP
#define VIENNAMINI_STEPPER_HPP

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
#include <map>

#include "viennamini/forwards.h"

namespace viennamini {

class stepper
{
private:
  typedef std::vector<numeric>                ValuesType;
  typedef std::map<std::size_t, ValuesType>   SegmentValuesType;
  typedef std::vector< std::vector< std::pair< std::size_t, numeric> > >  StepValuesType;

public:
  typedef ValuesType                          values_type;
  typedef SegmentValuesType                   segment_values_type;

  stepper                           (viennamini::device_handle& device);

  void         add                  (std::size_t segment_index, numeric const& start, numeric const& end, numeric const& delta);
  values_type  compute_value_range  (numeric const& start, numeric const& end, numeric const& delta);
  void         update               ();
  bool         apply_next           ();
  void         write                (std::ostream& stream = std::cout);
  std::size_t  size                 ();
  bool         empty                ();

private:
  viennamini::device_handle&  device_;
  SegmentValuesType           segment_values_;
  StepValuesType              step_values_;
};

} // viennamini

#endif


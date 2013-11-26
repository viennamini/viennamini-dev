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
  typedef std::pair< std::size_t, numeric>    StepEntryType;
  typedef std::vector< StepEntryType >        StepSetupType;
  typedef std::vector< StepSetupType >        StepValuesType;

public:
  typedef ValuesType                          values_type;
  typedef StepEntryType                       step_entry_type;
  typedef StepSetupType                       step_setup_type;
  typedef StepValuesType                      step_values_type;

  stepper                                           (segment_values& current_contact_potentials);

  void              add                             (std::size_t segment_index, numeric const& start, numeric const& end, numeric const& delta);
  values_type       compute_value_range             (numeric const& start, numeric const& end, numeric const& delta);
  void              update                          ();
  bool              apply_next                      ();
  std::size_t       get_current_step_id             ();
  step_setup_type&  get_step_setup                  (std::size_t step_id);
  step_setup_type&  get_current_step_setup          ();
  void              write                           (std::ostream& stream = std::cout);
  std::size_t       size                            ();
  bool              empty                           ();

private:
  StepValuesType              step_values_;
  StepValuesType::iterator    current_step_;
  segment_values            & current_contact_potentials_; 
};

} // viennamini

#endif


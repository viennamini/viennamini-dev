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

#include "viennamini/stepper.hpp"
#include "viennamini/utils/convert.hpp"

namespace viennamini {



class range_invalid_exception : public std::runtime_error {
public:
  range_invalid_exception(std::string const & str) : std::runtime_error(str) {}
};


stepper::stepper()
{
}

void stepper::add(std::size_t segment_index, numeric const& start, numeric const& end, numeric const& delta)
{
  ValuesType values = this->compute_value_range(start, end, delta);
  if(values.empty())
    throw range_invalid_exception("Invalid range: "+viennamini::convert<std::string>()(start)+" -> "+
                                                    viennamini::convert<std::string>()(end)  +", delta: "+
                                                    viennamini::convert<std::string>()(delta));
  this->add_impl(segment_index, values);
}

void stepper::add(std::size_t segment_index, numeric const& value)
{
  ValuesType values;
  values.push_back(value);
  this->add_impl(segment_index, values);
}

void stepper::add_impl(std::size_t segment_index, ValuesType& values)
{
  if(step_values_.empty())
  {
    for(ValuesType::iterator viter = values.begin(); viter != values.end(); viter++)
    {
      StepSetupType entries;
      entries.push_back(std::make_pair(segment_index, *viter));
      step_values_.push_back(entries);
    }
  }
  else
  {
    StepValuesType temp_step_values_ = step_values_;
    step_values_.clear();

    for(ValuesType::iterator viter = values.begin(); viter != values.end(); viter++)
    {
      for(StepValuesType::iterator iter = temp_step_values_.begin();
          iter != temp_step_values_.end(); iter++)
      {
        StepSetupType entries;
        entries.push_back(std::make_pair(segment_index, *viter));

        for(StepValuesType::value_type::iterator iter2 = iter->begin();
            iter2 != iter->end(); iter2++)
        {
          entries.push_back(std::make_pair(iter2->first, iter2->second));
        }

        step_values_.push_back(entries);
      }
    }
  }
  current_step_=step_values_.begin(); // start from scratch, if a new range variable is added
}

stepper::values_type  stepper::compute_value_range(numeric const& start, numeric const& end, numeric const& delta)
{
  stepper::values_type cont;
  for(numeric i = start; ; i+=delta)
  {
    if(std::fabs(i) < 1.0E-10) i = 0.0;
//    std::cout << start << " " << end << " " << delta << ": " << i << std::endl;
    cont.push_back(i);
    if(std::fabs(i-end)<1.0E-10) break;
  }
  return cont;
}

bool stepper::apply_next(segment_values& current_contact_potentials)
{
  if( (current_step_) == step_values_.end()) return false;

  current_contact_potentials.clear();
  for(StepValuesType::value_type::iterator iter = current_step_->begin();
      iter != current_step_->end(); iter++)
  {
//    std::cout << "applying at " << iter->first << " value " << iter->second << std::endl;
//    device_->set_contact_potential(iter->first, iter->second);
    current_contact_potentials[iter->first] = iter->second;
  }

  ++current_step_;
  return true;

}

// step id is a 1-based index, thus we compute the distance from begin
// to current, where the latter already points to the next step, yielding
// a one-based index
//
std::size_t stepper::get_current_step_id()
{
  return std::distance(step_values_.begin(), current_step_);
}

stepper::step_setup_type&   stepper::get_step_setup(std::size_t step_id)
{
  return step_values_[step_id];
}

stepper::step_setup_type&  stepper::get_current_step_setup()
{
  return this->get_step_setup(this->get_current_step_id()-1);
}

void stepper::write(std::ostream& stream)
{
  stream << "Writing stepping sequence: " << std::endl;
  stream << "  Sequence size: " << this->size() << std::endl;
  for(StepValuesType::iterator iter = step_values_.begin();
      iter != step_values_.end(); iter++)
  {
    stream << "    Simulation step: " << std::distance(step_values_.begin(), iter)+1;
    for(StepSetupType::iterator iter2 = iter->begin();
        iter2 != iter->end(); iter2++)
    {
       stream << " Segment: " << iter2->first << " - BC: " << iter2->second;
       if((iter2+1) != iter->end())
         stream << ", ";
    }
    stream << "\n";
  }
}

std::size_t stepper::size()
{
  return step_values_.size();
}

bool stepper::empty()
{
  return step_values_.empty();
}

void stepper::clear()
{
  step_values_.clear();
}

} // viennamini



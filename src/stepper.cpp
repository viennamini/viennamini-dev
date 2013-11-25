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
#include "viennamini/device.hpp"
#include "viennamini/utils/convert.hpp"

namespace viennamini {



class range_invalid_exception : public std::runtime_error {
public:
  range_invalid_exception(std::string const & str) : std::runtime_error(str) {}
};


stepper::stepper(viennamini::device_handle& device) : device_(device)
{
}

void stepper::add(std::size_t segment_index, numeric const& start, numeric const& end, numeric const& delta)
{
  ValuesType values = this->compute_value_range(start, end, delta);
  if(values.empty()) 
    throw range_invalid_exception("Invalid range: "+viennamini::convert<std::string>()(start)+" -> "+
                                                    viennamini::convert<std::string>()(end)  +", delta: "+
                                                    viennamini::convert<std::string>()(delta));
  segment_values_[segment_index]   = values;
}

stepper::values_type  stepper::compute_value_range(numeric const& start, numeric const& end, numeric const& delta)
{
  stepper::values_type cont;
  for(numeric i = start; i <= end; i+=delta)
  {
    cont.push_back(i);
  }
  return cont;
}

bool stepper::apply_next()
{


}

void stepper::update()
{
  step_values_.clear();

  std::vector< std::pair< std::size_t, numeric> >  cont;

  for(SegmentValuesType::iterator iter = segment_values_.begin();
      iter != segment_values_.end(); iter++)
  {
//    StepValuesType::value_type  entries(segment_values_.size());

//    step_values_.push_back(  );
    for(SegmentValuesType::mapped_type::iterator viter = iter->second.begin();
        viter != iter->second.end(); viter++)
    {
      cont.push_back(std::make_pair(iter->first, *viter));
    }


  }
}

void stepper::write(std::ostream& stream)
{
  std::size_t cnt = 1;
  for(SegmentValuesType::iterator iter = segment_values_.begin();
      iter != segment_values_.end(); iter++)
  {

//    stream << "id: " << cnt << " segment-id: " << it
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

} // viennamini


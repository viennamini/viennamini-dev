#ifndef VIENNAMINI_CONFIG_HPP
#define VIENNAMINI_CONFIG_HPP

#include <map>
#include <vector>

namespace viennamini {

struct Config
{
  typedef double Numeric;
  typedef std::vector<Numeric> Values;
  typedef std::map<std::size_t, Values > SegmentContacts;

  Numeric& acc_temperature() { return temperature; }

  void add_contact(std::size_t segment_index, Numeric value)
  {
    segment_contact_values[segment_index].push_back(value);
  }

  Values& get_contact_values(std::size_t segment_index)
  {
    return segment_contact_values[segment_index];
  }

private:
  Numeric temperature;
  SegmentContacts segment_contact_values;
};




} // viennamini



#endif



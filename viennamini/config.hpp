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
  typedef std::map<std::size_t, Numeric> SegmentContactWorkfunctions;

  Numeric& acc_temperature() { return temperature; }

  void add_contact(std::size_t segment_index, Numeric value, Numeric workfunction)
  {
    segment_contact_values[segment_index].push_back(value);
    segment_contact_workfunctions[segment_index] = workfunction;
  }

  Values& get_contact_values(std::size_t segment_index)
  {
    return segment_contact_values[segment_index];
  }

  Numeric get_workfunction(std::size_t segment_index)
  {
      return segment_contact_workfunctions[segment_index];
  }

private:
  Numeric temperature;
  SegmentContacts segment_contact_values;
  SegmentContactWorkfunctions segment_contact_workfunctions;
};




} // viennamini



#endif



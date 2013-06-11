#ifndef VIENNAMINI_CONFIG_HPP
#define VIENNAMINI_CONFIG_HPP

#include <map>
#include <vector>

namespace viennamini {



struct Config
{
  typedef double Numeric;
  typedef std::size_t Index;
  typedef std::vector<Numeric> Values;
  typedef std::map<std::size_t, Values > SegmentContacts;
  typedef std::map<std::size_t, Numeric> SegmentContactWorkfunctions;

  Config()
  {
    local_temperature = 300.0; // K
    local_nonlinear_iterations = 50;
    local_nonlinear_breaktol = 1.E-3;
    local_linear_breaktol = 1.E-14;
    local_linear_iterations = 700;
    local_damping = 0.3;
  }

  Numeric& temperature() { return local_temperature; }
  Index& nonlinear_iterations() { return local_nonlinear_iterations; }
  Numeric& nonlinear_breaktol() { return local_nonlinear_breaktol; }
  Index& linear_iterations() { return local_linear_iterations; }
  Numeric& linear_breaktol() { return local_linear_breaktol; }
  Numeric& dampening() { return local_damping; }

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
  Numeric local_temperature;

  std::size_t local_nonlinear_iterations;
  Numeric     local_nonlinear_breaktol;
  std::size_t local_linear_iterations;
  Numeric     local_linear_breaktol;
  Numeric     local_damping;

  SegmentContacts segment_contact_values;
  SegmentContactWorkfunctions segment_contact_workfunctions;
};




} // viennamini



#endif



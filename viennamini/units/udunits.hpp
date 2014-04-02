#ifndef VIENNAMINI_UNITS_UDUNITS_HPP
#define VIENNAMINI_UNITS_UDUNITS_HPP

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

extern "C"{
#include "udunits2.h"
}

#include "viennamini/forwards.h"

namespace viennamini {

class udunits_exception : public std::runtime_error {
public:
  udunits_exception(std::string const & str) : std::runtime_error(str) {}
};

struct udunits
{
private:
  typedef std::map<std::string, ut_unit*>   UnitsLookupType;
 
public:
  typedef UnitsLookupType                   units_lookup_type;

  udunits() : unit_system_(ut_read_xml("../../auxiliary/udunits2.xml"))
  {
    units_lookup_[viennamini::unit::si::kelvin()]                 = ut_get_unit_by_symbol(unit_system_, "K");
    units_lookup_[viennamini::unit::si::carrier_concentration()]  = ut_parse(unit_system_, "1/m-3", UT_UTF8);
  }

  ~udunits()
  {
    ut_free_system(unit_system_);
  }

  void convert(viennamini::numeric& value, std::string const& source_unit, std::string const& target_unit)
  {
    std::cout << "converting: " << value << " " << source_unit << " " << target_unit << std::endl;
//    check_unit_support(source_unit, target_unit);
//    cv_converter* converter = ut_get_converter(units_lookup_[source_unit], units_lookup_[target_unit]);
//    value = cv_convert_double(converter, value);
  }

  void check_unit_support(std::string const& source_unit, std::string const& target_unit)
  {
    if(units_lookup_.find(source_unit) == units_lookup_.end()) throw udunits_exception("Source unit \""+source_unit+"\" is not available!");
    if(units_lookup_.find(target_unit) == units_lookup_.end()) throw udunits_exception("Target unit \""+target_unit+"\" is not available!");
  }

private:
  ut_system*          unit_system_;
  units_lookup_type   units_lookup_;
};

} // viennamini

#endif 


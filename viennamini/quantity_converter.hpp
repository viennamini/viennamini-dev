#ifndef VIENNAMINI_QUANTITYCONVERTER_HPP
#define VIENNAMINI_QUANTITYCONVERTER_HPP

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

#ifdef VIENNAMINI_WITH_UNITS
  #include "viennamini/units/udunits.hpp"
#endif

#include <map>

#include "viennamini/forwards.h"

namespace viennamini {

class quantity_converter_exception : public std::runtime_error {
public:
  quantity_converter_exception(std::string const & str) : std::runtime_error(str) {}
};

struct quantity_converter
{
private:
  typedef std::map<std::string, std::string>    QuantityUnitLookupType;

public:
  quantity_converter()
  {
    quantity_unit_lookup_[viennamini::id::donor_doping()]           = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::acceptor_doping()]        = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::temperature()]            = viennamini::unit::si::kelvin();
    quantity_unit_lookup_[viennamini::id::potential()]              = viennamini::unit::si::volt();
    quantity_unit_lookup_[viennamini::id::relative_permittivity()]  = viennamini::unit::none();
//    quantity_unit_lookup_[viennamini::id::electron_mobility()]      = viennamini::unit::none();
//    quantity_unit_lookup_[viennamini::id::hole_mobility()]          = viennamini::unit::none();
  }

  void run(std::string const& quantity_name, viennamini::numeric& value, std::string const& unit)
  {
    if(!is_available(quantity_name))  throw quantity_converter_exception("Quantity \""+quantity_name+"\" is not registered!");

    if(!is_conform(quantity_name, unit))
    {
    #ifdef VIENNAMINI_WITH_UNITS
      // the unit of the quantity is wrong, convert the quantity to the required unit
      udunits_.convert(value, unit, get_conform_unit(quantity_name));
    #else
      // we cannot automatically convert it, so throw an exception to ensure
      // that the simulation is not further continued, as the unit is wrong
      //
      throw quantity_converter_exception("Quantity \""+quantity_name+"\" has non-conform unit: "+
        unit+ " (should be: "+get_conform_unit(quantity_name)+" )");
    #endif
    }
  }

  void run(std::string const& quantity_name, viennamini::sparse_values& values, std::string const& unit)
  {
  #ifdef VIENNAMINI_WITH_UNITS

  #endif
  }

  void run(std::string const& quantity_name, viennamini::dense_values& values, std::string const& unit)
  {
  #ifdef VIENNAMINI_WITH_UNITS

  #endif
  }

  std::string get_conform_unit(std::string const& quantity_name)
  {
    return quantity_unit_lookup_[quantity_name];
  }

  bool is_available(std::string const& quantity_name)
  {
    return quantity_unit_lookup_.find(quantity_name) != quantity_unit_lookup_.end();
  }

  bool is_conform(std::string const& quantity_name, std::string const& unit)
  {
    return quantity_unit_lookup_.at(quantity_name) == unit;
  }

private:
  QuantityUnitLookupType  quantity_unit_lookup_;

#ifdef VIENNAMINI_WITH_UNITS
  viennamini::udunits   udunits_;
#endif
};

} // viennamini

#endif 


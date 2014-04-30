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

#include "viennamini/quantity_converter.hpp"

namespace viennamini {

  quantity_converter::quantity_converter(std::string const& filename)
  #ifdef VIENNAMINI_WITH_UNITS
    : udunits_(filename)
  #endif
  {
    quantity_unit_lookup_[viennamini::id::donor_doping()]           = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::acceptor_doping()]        = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::intrinsic_carrier()]      = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::electron_concentration()] = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::hole_concentration()]     = viennamini::unit::si::carrier_concentration();
    quantity_unit_lookup_[viennamini::id::temperature()]            = viennamini::unit::si::temperature();
    quantity_unit_lookup_[viennamini::id::potential()]              = viennamini::unit::si::potential();
    quantity_unit_lookup_[viennamini::id::electron_mobility()]      = viennamini::unit::si::mobility();
    quantity_unit_lookup_[viennamini::id::hole_mobility()]          = viennamini::unit::si::mobility();
    quantity_unit_lookup_[viennamini::id::relative_permittivity()]  = viennamini::unit::none();
  }

  void quantity_converter::run(std::string const& quantity_name, viennamini::numeric& value, std::string const& unit)
  {
    if(!is_available(quantity_name))  throw quantity_converter_exception("Quantity \""+quantity_name+"\" is not registered with the quantity converter!");

    if(!is_conform(quantity_name, unit))
    {
    #ifdef VIENNAMINI_WITH_UNITS
      // the unit of the quantity is wrong, convert the quantity to the required unit
    #ifdef VIENNAMINI_VERBOSE
      viennamini::numeric old = value;
    #endif
      try {
        udunits_.convert(value, unit, get_conform_unit(quantity_name));
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "Converted \"" << quantity_name << "\" " << old << " \"" << unit << "\" --> " << value << " \"" << get_conform_unit(quantity_name) << "\"" << std::endl;
      #endif
      }
      catch (std::exception const& e)
      {
        throw quantity_converter_exception("Quantity \"" + quantity_name + "\" has non-conform unit \"" +
          unit + "\": must be based on \"" + get_conform_unit(quantity_name));
      }
    #else
      // we cannot automatically convert it, so throw an exception to ensure
      // that the simulation is not further continued, as the unit is wrong
      //
      throw quantity_converter_exception("Quantity \""+quantity_name+"\" has non-conform unit: "+
        unit+ " (should be: "+get_conform_unit(quantity_name)+" ).\nBuild ViennaMini with Unit Support to enable automatic conversions!");
    #endif
    }
  }

  void quantity_converter::run(std::string const& quantity_name, viennamini::sparse_values& values, std::string const& unit)
  {
    if(!is_available(quantity_name))  throw quantity_converter_exception("Quantity \""+quantity_name+"\" is not registered with the quantity converter!");

    if(!is_conform(quantity_name, unit))
    {
    #ifdef VIENNAMINI_WITH_UNITS
      // the unit of the quantity is wrong, convert the quantity to the required unit
      try {
        udunits_.convert(values, unit, get_conform_unit(quantity_name));
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "Converted \"" << quantity_name << "\" " << unit << "\" --> " << get_conform_unit(quantity_name) << "\"" << std::endl;
      #endif
      }
      catch (std::exception const& e)
      {
        throw quantity_converter_exception("Quantity \"" + quantity_name + "\" has non-conform unit \"" +
          unit + "\": must be based on \"" + get_conform_unit(quantity_name));
      }
    #else
      // we cannot automatically convert it, so throw an exception to ensure
      // that the simulation is not further continued, as the unit is wrong
      //
      throw quantity_converter_exception("Quantity \""+quantity_name+"\" has non-conform unit: "+
        unit+ " (should be: "+get_conform_unit(quantity_name)+" ).\nBuild ViennaMini with Unit Support to enable automatic conversions!");
    #endif
    }
  }

  void quantity_converter::run(std::string const& quantity_name, viennamini::dense_values& values, std::string const& unit)
  {
    if(!is_available(quantity_name))  throw quantity_converter_exception("Quantity \""+quantity_name+"\" is not registered with the quantity converter!");

    if(!is_conform(quantity_name, unit))
    {
    #ifdef VIENNAMINI_WITH_UNITS
      // the unit of the quantity is wrong, convert the quantity to the required unit
      try {
        udunits_.convert(values, unit, get_conform_unit(quantity_name));
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "Converted \"" << quantity_name << "\" " << unit << "\" --> " << get_conform_unit(quantity_name) << "\"" << std::endl;
      #endif
      }
      catch (std::exception const& e)
      {
        throw quantity_converter_exception("Quantity \"" + quantity_name + "\" has non-conform unit \"" +
          unit + "\": must be based on \"" + get_conform_unit(quantity_name));
      }
    #else
      // we cannot automatically convert it, so throw an exception to ensure
      // that the simulation is not further continued, as the unit is wrong
      //
      throw quantity_converter_exception("Quantity \""+quantity_name+"\" has non-conform unit: "+
        unit+ " (should be: "+get_conform_unit(quantity_name)+" ).\nBuild ViennaMini with Unit Support to enable automatic conversions!");
    #endif
    }
  }

  std::string quantity_converter::get_conform_unit(std::string const& quantity_name)
  {
    if(!is_available(quantity_name))  throw quantity_converter_exception("Quantity \""+quantity_name+"\" is not registered with the quantity converter!");
    return quantity_unit_lookup_[quantity_name];
  }

  bool quantity_converter::is_available(std::string const& quantity_name)
  {
    return quantity_unit_lookup_.find(quantity_name) != quantity_unit_lookup_.end();
  }

  bool quantity_converter::is_conform(std::string const& quantity_name, std::string const& unit)
  {
    return quantity_unit_lookup_.at(quantity_name) == unit;
  }

} // viennamini

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

/** @file viennamini/quantity_converter.hpp
    @brief Automatic quantity-check and unit-conversion facility class
*/

#ifdef VIENNAMINI_WITH_UNITS
  #include "viennamini/units/udunits.hpp"
#endif

#include <map>

#include "viennamini/forwards.h"

namespace viennamini {

/** @brief An exception class for the quantity_converter class
  */
class quantity_converter_exception : public std::runtime_error {
public:
  quantity_converter_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief The class provides a high level interface for evaluating whether a
  *        quantity has the expected/required unit. Provides automatic error handling
  *        and conversion routines.
  */
struct quantity_converter
{
private:
  typedef std::map<std::string, std::string>    QuantityUnitLookupType;

public:

/** @brief The constructor forwards the filename pointing to an XML-based
  *        unit database to the unit backend.
  *        Also a lookup-table mapping quantity keys with expected physical units is populated,
  *        which is later used during the execution to drive the conversion/check processes.
  *
  * @param filename The XML input filename pointing to a unit database
  */
  quantity_converter(std::string const& filename = "");

/** @brief Investigate (and possible convert) a quantity holding a single scalar value
  *
  * @param quantity_name The quantity name used to lookup the required unit in the local lookup table.
  * @param value         The value of the quantity which is possibly converted relative to the required unit.
  * @param unit          The unit of the value, used to determine whether the quantity offers the correct unit.
  */
  void run(std::string const& quantity_name, viennamini::numeric& value, std::string const& unit);

/** @brief Investigate (and possible convert) a quantity holding values in a sparsely-indexed container
  *
  * @param quantity_name The quantity name used to lookup the required unit in the local lookup table.
  * @param values        The sparsely-indexed value container of the quantity which are possibly converted relative to the required unit.
  * @param unit          The unit of the value, used to determine whether the quantity offers the correct unit.
  */
  void run(std::string const& quantity_name, viennamini::sparse_values& values, std::string const& unit);

/** @brief Investigate (and possible convert) a quantity holding values in a densely-indexed container
  *
  * @param quantity_name The quantity name used to lookup the required unit in the local lookup table.
  * @param values        The densely-indexed value container of the quantity which are possibly converted relative to the required unit.
  * @param unit          The unit of the value, used to determine whether the quantity offers the correct unit.
  */
  void run(std::string const& quantity_name, viennamini::dense_values& values, std::string const& unit);

/** @brief Method returns the required unit string for a given quantity
  *
  * @param quantity_name The name of the quantity for which the conformal unit is to be extracted
  * @return              The conformal unit string
  */
  std::string get_conform_unit(std::string const& quantity_name);

/** @brief Method evaluates whether a quantity name is registered within the quantity converter
  *        i.e. the quantity is mapped to a conformal unit in the local lookup table
  *
  * @param quantity_name The name of the quantity to be looked-up
  * @return              Boolean returning 'true' if the quantity is registered, 'false' otherwise.
  */
  bool is_available(std::string const& quantity_name);

/** @brief Method evaluates whether a unit is the conformal unit of a quantity
  *        i.e. the quantity's unit offers the unit required by ViennaMini
  *
  * @param quantity_name The name of the quantity to be looked-up
  * @param unit          The unit of the quantity to be evaluated
  * @return              Boolean returning 'true' if the quantity is registered, 'false' otherwise.
  */
  bool is_conform(std::string const& quantity_name, std::string const& unit);

private:
  QuantityUnitLookupType  quantity_unit_lookup_;

#ifdef VIENNAMINI_WITH_UNITS
  viennamini::udunits   udunits_;
#endif
};

} // viennamini

#endif

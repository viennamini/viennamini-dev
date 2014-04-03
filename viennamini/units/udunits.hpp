#ifndef VIENNAMINI_UNITS_UDUNITS_HPP
#define VIENNAMINI_UNITS_UDUNITS_HPP

/* =======================================================================
   Copyright (c) 2011-2014, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */

/** @file viennamini/units/udunits.hpp
    @brief Interface class for the udunits library
*/

extern "C"{
#include "udunits2.h"
}

#include "viennamini/forwards.h"

namespace viennamini {

/** @brief An exception class for the udunits class
  */
class udunits_exception : public std::runtime_error {
public:
  udunits_exception(std::string const & str) : std::runtime_error(str) {}
};

/** @brief The udunits class provides an interface to the udunits library
  */
struct udunits
{
public:

  /** @brief The constructor expects a filename pointing to a udunits-valid XML file containing the unit system
  *          and forwards it to the udunits' backend by instantiating a 'unit_system' object
  *
  * @param filename   The filename to be loaded
  */
  udunits(std::string const& filename);

  /** @brief The destructor releases the 'unit_system' object
  *
  */
  ~udunits();

  /** @brief The function scales a numeric value from a source unit to a physical unit, e.g., 1 mV -> 0.001 V
  *
  * @param value       The single numerical value (i.e. double) to be scaled
  * @param source_unit The original unit associated with the value
  * @param target_unit The target unit to which the value has to be converted to
  */
  void convert(viennamini::numeric& value, std::string const& source_unit, std::string const& target_unit);


  /** @brief The function scales a sparse container of values (i.e. map<size_t,double>)
  *          from a source unit to a physical unit, e.g., {2:1, 1:1, 5:1} mV -> {2:0.001, 1:0.001, 5:0.001} V
  *
  * @param values      A sparse container (i.e. map) holding values which are all to be converted
  * @param source_unit The original unit associated with the values
  * @param target_unit The target unit to which the values have to be converted to
  */
  void convert(viennamini::sparse_values& values, std::string const& source_unit, std::string const& target_unit);

  /** @brief The function scales a dense container of values (i.e. vector<double>)
  *          from a source unit to a physical unit, e.g., {1, 2, 3} mV -> {0.001, 0.002, 0.003} V
  *
  * @param values      A dense container (i.e. vector) holding values which are all to be converted
  * @param source_unit The original unit associated with the values
  * @param target_unit The target unit to which the values have to be converted to
  */
  void convert(viennamini::dense_values& values, std::string const& source_unit, std::string const& target_unit);

private:

  /** @brief An auxiliary function generating a udunit value converter
  *
  * @param source_unit The unit of the input value
  * @param target_unit The unit of the target value
  */
  cv_converter* get_converter(std::string const& source_unit, std::string const& target_unit);

  /** @brief An auxiliary function evaluating the string-based unit parsing error code
  *          returned by udunits' 'ut_parse' method
  *
  * @param unit The unit string which has been used as a parser input
  */
  void investigate_parse_error(std::string const& unit);

  /** @brief An auxiliary function evaluating the conversion error raised by
  *          udunits' conversion method
  *
  * @param source_unit The unit of the input value
  * @param target_unit The unit of the target value
  */
  void investigate_conversion_error(std::string const& source_unit, std::string const& target_unit);

private:

  /** @brief A pointer to the udunits' unit system facility.
  *
  */
  ut_system*          unit_system_;
};

} // viennamini

#endif


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

namespace viennamini {

  udunits::udunits(std::string const& filename) 
  {
    unit_system_ = ut_read_xml(filename.c_str());
    ut_status stat = ut_get_status();
    if(stat == UT_OPEN_ARG)   throw udunits_exception("Could not open unit database!");
    else if(stat == UT_OS)    throw udunits_exception("Unit database encountered operating system error!");
    else if(stat == UT_PARSE) throw udunits_exception("Unit database could not process unit XML file!");

    ut_set_error_message_handler(&ut_ignore);
  }

  udunits::~udunits()
  {
    ut_free_system(unit_system_);
  }

  void udunits::convert(viennamini::numeric& value, std::string const& source_unit, std::string const& target_unit)
  {
    cv_converter* converter = this->get_converter(source_unit, target_unit);
    value = cv_convert_double(converter, value);
    cv_free(converter);
  }

  void udunits::convert(viennamini::sparse_values& values, std::string const& source_unit, std::string const& target_unit)
  {
    cv_converter* converter = this->get_converter(source_unit, target_unit);
    for(viennamini::sparse_values::iterator iter = values.begin(); iter != values.end(); iter++)
      iter->second = cv_convert_double(converter, iter->second);
    cv_free(converter);
  }

  void udunits::convert(viennamini::dense_values& values, std::string const& source_unit, std::string const& target_unit)
  {
    cv_converter* converter = this->get_converter(source_unit, target_unit);
    for(viennamini::dense_values::iterator iter = values.begin(); iter != values.end(); iter++)
      *iter = cv_convert_double(converter, *iter);
    cv_free(converter);
  }

  cv_converter* udunits::get_converter(std::string const& source_unit, std::string const& target_unit)
  {
    ut_unit* udunit_source = ut_parse(unit_system_, source_unit.c_str(), UT_UTF8);
    if(!udunit_source) investigate_parse_error(source_unit);

    ut_unit* udunit_target = ut_parse(unit_system_, target_unit.c_str(), UT_UTF8);
    if(!udunit_target) investigate_parse_error(target_unit);

    cv_converter* converter = ut_get_converter(udunit_source, udunit_target);
    if(!converter) investigate_conversion_error(source_unit, target_unit);
    return converter;
  }

  void udunits::investigate_parse_error(std::string const& unit)
  {
    std::string err_msg = "Error parsing unit \""+unit+"\": ";
    ut_status status = ut_get_status();
    if(status == UT_BAD_ARG) throw udunits_exception(err_msg+"Unit or system is NULL!");
    else
    if(status == UT_SYNTAX)  throw udunits_exception(err_msg+"Unit string contains a syntax error!");
    else
    if(status == UT_UNKNOWN) throw udunits_exception(err_msg+"Unit string contains an unknown identifier!");
    else                     throw udunits_exception(err_msg+"An unknown error occured!");
  }

  void udunits::investigate_conversion_error(std::string const& source_unit, std::string const& target_unit)
  {
    std::string err_msg = "Error converting from \""+source_unit+"\" to \""+target_unit+"\": ";
    ut_status status = ut_get_status();
    if(status == UT_BAD_ARG) throw udunits_exception(err_msg+"One of the units is NULL!");
    else
    if(status == UT_NOT_SAME_SYSTEM)  throw udunits_exception(err_msg+"The units are from different unit systems!");
    else
    if(status == UT_SUCCESS) throw udunits_exception(err_msg+"Units cannot be converted!");
    else
    if(status == UT_MEANINGLESS) throw udunits_exception(err_msg+"Units cannot be converted!");
    else                     throw udunits_exception(err_msg+"An unknown error occured!");
  }

} // viennamini

#endif


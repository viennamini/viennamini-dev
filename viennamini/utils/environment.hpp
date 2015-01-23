#ifndef VIENNAMINI_UTILS_ENVIRONMENT_HPP
#define VIENNAMINI_UTILS_ENVIRONMENT_HPP

/* =======================================================================
   Copyright (c) 2011-2015, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */

// System includes
//
#include <cstdlib>

namespace viennamini {

std::string extract_environment_variable( std::string const& key )
{
    char * val = getenv( key.c_str() );
    return val == NULL ? std::string("") : std::string(val);
}

} // viennamini

#endif

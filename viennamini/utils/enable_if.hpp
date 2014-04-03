#ifndef VIENNAMINI_UTIL_ENABLEIF_HPP
#define VIENNAMINI_UTIL_ENABLEIF_HPP

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

/** @file viennamini/util/enable_if.hpp
    @brief Simple enable-if variant that uses the SFINAE pattern
*/

namespace viennamini
{
    /** @brief Simple enable-if variant that uses the SFINAE pattern */
    template <bool b, class T = void>
    struct enable_if
    {
      typedef T   type;
    };

    template <class T>
    struct enable_if<false, T> {};

} // viennamini

#endif


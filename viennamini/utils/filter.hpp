#ifndef VIENNAMINI_UTILS_FILTER_HPP
#define VIENNAMINI_UTILS_FILTER_HPP

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

namespace viennamini {

struct any_filter
{
    template <typename T>
    bool operator()(T const & t)       { return true; }

    template <typename T>
    bool operator()(T const & t) const { return true; }
};


} // viennamini

#endif


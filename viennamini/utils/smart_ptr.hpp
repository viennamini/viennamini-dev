#ifndef VIENNAMINI_UTILS_SMARTPTR_HPP
#define VIENNAMINI_UTILS_SMARTPTR_HPP

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

#include <memory>

namespace viennamini {

#ifdef VIENNAMINI_WITH_CXX11
  template<typename T>
  class smart_ptr : public std::unique_ptr<T> 
  {
  public:
    smart_ptr(T * ptr) : std::unique_ptr<T>(ptr) {}
  };
#else
  template<typename T>
  class smart_ptr : public std::auto_ptr<T> 
  {
  public:
    smart_ptr(T * ptr) : std::auto_ptr<T>(ptr) {}
  };
#endif

} // viennamini

#endif


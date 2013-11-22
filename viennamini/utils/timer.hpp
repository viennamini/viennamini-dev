
#ifndef VIENNAMINI_UTILS_TIMER_HPP
#define VIENNAMINI_UTILS_TIMER_HPP

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

#include <iostream>

namespace viennamini {

#ifdef _WIN32

#define WINDOWS_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max

//! timer class
class timer
{
public:
   //! default constructor
   timer()
   {
      QueryPerformanceFrequency(&freq);
   }
   //! start the timer
   void start()
   {
      QueryPerformanceCounter((LARGE_INTEGER*) &start_time);
   }
   //! retrieve the timer count
   double get() const
   {
      LARGE_INTEGER  end_time;
      QueryPerformanceCounter((LARGE_INTEGER*) &end_time);
      return (static_cast<double>(end_time.QuadPart) - static_cast<double>(start_time.QuadPart)) / static_cast<double>(freq.QuadPart);
   }


private:
   //! timer states
   LARGE_INTEGER freq;
   LARGE_INTEGER start_time;
};

#else

#include <sys/time.h>

//! timer class
class timer
{
public:
   //! default constructor
   timer() : ts(0)
   {}
   //! start the timer
   void start()
   {
      struct timeval tval;
      gettimeofday(&tval, NULL);
      ts = tval.tv_sec * 1000000 + tval.tv_usec;
   }
   //! retrieve the timer count
   double get() const
   {
      struct timeval tval;
      gettimeofday(&tval, NULL);
      unsigned long end_time = tval.tv_sec * 1000000 + tval.tv_usec;

      return static_cast<double>(end_time-ts) / 1000000.0;
   }

private:
   //! state
   unsigned long ts;
};


#endif

} // end namespace viennamini

#endif



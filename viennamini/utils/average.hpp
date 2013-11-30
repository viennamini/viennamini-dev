#ifndef VIENNAMINI_UTILS_AVERAGE_HPP
#define VIENNAMINI_UTILS_AVERAGE_HPP

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

/** @brief A functor which computes the arithmetic average of all entries in a container.
 *
 * The container is required to provide STL-compatible forward-iterators and a member function size() returning the number of elements.
 */
struct arithmetic_averaging
{
  typedef double result_type;

  template <typename ContainerType>
  result_type operator()(ContainerType const & cont) const
  {
    result_type ret = 0;
    for (typename ContainerType::const_iterator cit  = cont.begin();
                                                cit != cont.end();
                                              ++cit)
          ret += *cit / cont.size();

    return ret;
  }
};

/** @brief A functor which computes the geometric average of all entries in a container.
 *
 * The container is required to provide STL-compatible forward-iterators and a member function size() returning the number of elements.
 * Note that all entries need to be non-negative for even container sizes.
 */
struct geometric_averaging
{
  typedef double result_type;

  template <typename ContainerType>
  result_type operator()(ContainerType const & cont) const
  {
    result_type ret = 1.0;
    for (typename ContainerType::const_iterator cit  = cont.begin();
                                                cit != cont.end();
                                              ++cit)
    {
      ret *= std::pow(*cit, 1.0 / cont.size());
    }

    return ret;
  }
};

} // viennamini

#endif


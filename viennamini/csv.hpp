
#ifndef VIENNAMINI_CSV_HPP
#define VIENNAMINI_CSV_HPP

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

#include <boost/numeric/ublas/matrix_sparse.hpp>

#include "viennamini/forwards.h"

namespace viennamini
{

struct csv
{
private:
  typedef boost::numeric::ublas::coordinate_matrix<numeric>   DatabaseType;

public:
  typedef DatabaseType                          database_type;

  csv()
  {
  }

  csv(std::size_t m, std::size_t n) : database_(m, n, m*n)
  {
  }

  void resize(std::size_t m, std::size_t n)
  {
    database_.resize(m, n, true);
  }

  void add_column(std::string name)
  {
  }
  
//  values_type& get_column(std::string name)
//  {
//    return database_[name];
//  }
  
  void write(std::ostream& stream = std::cout)
  {
//    for(DatabaseType::iterator iter = database_.begin();
//        iter != database_.end(); iter++)
//    {
//      
//    }
  }
  
private:
  DatabaseType database_;
};

} // viennamini

#endif


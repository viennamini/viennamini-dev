
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

#include <vector>

#include "viennamini/forwards.h"

namespace viennamini
{

struct csv
{
private:
  typedef std::vector<std::string>    HeaderLineType;
  typedef std::vector<numeric>        DataLineType;
  typedef std::vector<DataLineType>   MatrixType;

public:
  typedef HeaderLineType              header_line_type;
  typedef DataLineType                data_line_type;

  csv()
  {
  }

  void set_header(header_line_type& header_line)
  {
    header_.resize(header_line.size());
    std::copy(header_line.begin(), header_line.end(), header_.begin());
  }
  
  void add_line(data_line_type& data_line)
  {
    matrix_.push_back(data_line);
  }

  void clear()
  {
    header_.clear();
    matrix_.clear();
  }

  void write(std::string const& filename)
  {
    std::ofstream stream(filename.c_str());
    this->write(stream);
    stream.close();
  }

  void write(std::ostream& stream)
  {
    for(HeaderLineType::iterator iter = header_.begin(); 
        iter != header_.end(); iter++)
    {
      stream << *iter << "  ";
    }
    stream << "\n";

    for(MatrixType::iterator line_iter = matrix_.begin();
        line_iter != matrix_.end(); line_iter++)
    {
      for(DataLineType::iterator col_iter = line_iter->begin();
          col_iter != line_iter->end(); col_iter++)
      {
        stream << *col_iter << "  ";
      }
      stream << "\n";
    }
  }
  
private:
  HeaderLineType  header_;
  MatrixType      matrix_;
};

} // viennamini

#endif


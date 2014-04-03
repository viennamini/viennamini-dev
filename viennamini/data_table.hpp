
#ifndef VIENNAMINI_DATATABLE_HPP
#define VIENNAMINI_DATATABLE_HPP

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

class data_table_exception : public std::runtime_error {
public:
  data_table_exception(std::string const & str) : std::runtime_error(str) {}
};

struct data_table
{
private:
  typedef std::map<std::size_t, std::string>  IndexStringMapType;
  typedef std::map<std::string, std::size_t>  StringIndexMapType;
  typedef std::vector<numeric>        ColumnType;
  typedef std::vector<ColumnType>     MatrixType;

public:
  typedef ColumnType                  column_type;
  typedef MatrixType                  matrix_type;

  data_table()
  {
  }

  void new_column(std::string const& column_name)
  {
    matrix_.push_back(ColumnType());
    index_key_map_[matrix_.size()-1] = column_name;
    key_index_map_[column_name] = matrix_.size()-1;
  }

  void add(std::string column_name, numeric value)
  {
    matrix_[key_index_map_[column_name]].push_back(value);
  }

  std::string get_column_name(std::size_t column_index)
  {
    return index_key_map_[column_index];
  }

  column_type& get_column(std::string column_name)
  {
    return matrix_[key_index_map_[column_name]];
  }

  void clear()
  {
    index_key_map_.clear();
    key_index_map_.clear();
    matrix_.clear();
  }

  std::size_t column_size()
  {
    return matrix_.size();
  }

  void write(std::string const& filename)
  {
    std::ofstream stream(filename.c_str());
    this->write(stream);
    stream.close();
  }

  void write(std::ostream& stream)
  {
    // write the header, i.e., the column names
    //
    stream << "# ";
    for(std::size_t col_i = 0; col_i < matrix_.size(); col_i++)
    {
      stream << index_key_map_[col_i] << "  ";
    }
    stream << "\n";

    // write the data. expects currently that all columns have the same size
    //
    for(std::size_t i = 0; i < matrix_.front().size(); i++)
    {
      for(MatrixType::iterator col_iter = matrix_.begin();
          col_iter != matrix_.end(); col_iter++)
      {
        stream << (*col_iter)[i] << "  ";
      }
      stream << "\n";
    }
  }

private:
  IndexStringMapType  index_key_map_;
  StringIndexMapType  key_index_map_;
  MatrixType          matrix_;
};

} // viennamini

#endif


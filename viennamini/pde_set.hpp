#ifndef VIENNAMINI_PDESET_HPP
#define VIENNAMINI_PDESET_HPP

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

#include "viennamini/forwards.h"

#include "viennamath/expression.hpp"

namespace viennamini {

class pde_set
{
private:
  typedef std::vector<std::string>                  IDsType;
  typedef std::vector<viennamath::function_symbol>  FunctionSymbolsType;

public:
  typedef IDsType                   ids_type;
  typedef FunctionSymbolsType       function_symbols_type;

  pde_set() {}

  virtual ~pde_set() {}

  virtual std::string info() = 0;

  viennamath::equation  & equation        () { return equation_; }
  ids_type              & dependencies    () { return dependencies_; }
  ids_type              & unknowns        () { return unknowns_;     }
  function_symbols_type & function_symbols() { return function_symbols_; }

protected:
  void add_dependency(std::string obj) { dependencies_.push_back(obj); }
  void add_unknown   (std::string obj) { unknowns_.push_back(obj); }

private:
  viennamath::equation  equation_;
  IDsType               dependencies_;
  IDsType               unknowns_;
  FunctionSymbolsType   function_symbols_;
};

} // viennamini


#endif


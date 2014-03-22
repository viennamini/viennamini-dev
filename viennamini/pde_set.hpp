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
#include "viennamini/pde.hpp"

#include "viennamath/expression.hpp"

#include <set>

namespace viennamini {

/*

  TODO


*/

class pde_set_exception : public std::runtime_error {
public:
  pde_set_exception(std::string const & str) : std::runtime_error(str) {}
};

class pde_set
{
private:
  typedef std::vector<std::string>                  IDsType;
  typedef std::vector<pde>                          PDEsType;

protected:
  typedef viennamath::function_symbol               FunctionSymbolType;

public:
  typedef IDsType                   ids_type;
  typedef PDEsType                  pdes_type;

  pde_set() {}

  virtual ~pde_set() {}

  virtual std::string info() = 0;

  virtual pdes_type get_pdes() = 0;

  virtual bool is_linear() = 0;

  viennamath::equation  & equation        () { return equation_; }
  ids_type              & dependencies    () { return dependencies_; }
  ids_type              & unknowns        () { return unknowns_;     }

  bool unknown_supports_role(std::string const& unknown, viennamini::role::segment_role_ids segment_role) 
  { 
    if(unknown_role_lookup_.find(unknown) == unknown_role_lookup_.end())
      throw viennamini::pde_set_exception("Unknown \""+unknown+"\" is missing information regarding its support for specific segment-roles!");
    return unknown_role_lookup_[unknown].find(segment_role) != unknown_role_lookup_[unknown].end();
  }

  void register_quantity(std::string const& quantity_name, std::size_t quantity_id)
  {
    quantity_name_id_[quantity_name] = quantity_id;
  }

protected:
  void add_dependency  (std::string dependency) { dependencies_.push_back(dependency); }
  void add_unknown     (std::string unknown)    { unknowns_.push_back(unknown); }

  void add_role_support(std::string unknown, viennamini::role::segment_role_ids segment_role)    
  { 
    unknown_role_lookup_[unknown].insert(segment_role);
  }

  std::size_t get_quantity_id(std::string const& quantity_name) 
  { 
    if(quantity_name_id_.find(quantity_name) == quantity_name_id_.end())
      throw viennamini::pde_set_exception("Quantity \""+quantity_name+"\" has not been registered!");
    return quantity_name_id_[quantity_name];
  }

private:
  viennamath::equation  equation_;
  IDsType               dependencies_;
  IDsType               unknowns_;

  std::map<std::string, std::set<viennamini::role::segment_role_ids> >  unknown_role_lookup_;
  std::map<std::string, std::size_t>    quantity_name_id_;
};

} // viennamini


#endif


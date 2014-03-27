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
#include "viennamini/quantity_generator.hpp"
#include "viennamini/contact_model.hpp"

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
  typedef std::map<std::string, viennamini::quantity_generator*> InitialGuessLookupType;

protected:
  typedef viennamath::function_symbol               FunctionSymbolType;

public:
  typedef IDsType                   ids_type;
  typedef PDEsType                  pdes_type;

  pde_set() {}

  virtual ~pde_set() 
  {
    for(InitialGuessLookupType::iterator iter = initial_guess_lookup_.begin();
        iter != initial_guess_lookup_.end(); iter++)
    {
      if(iter->second) delete iter->second;
    }
  }

  virtual std::string info() = 0;

  virtual pdes_type get_pdes() = 0;

  virtual bool is_linear() = 0;

  viennamath::equation  & equation        () { return equation_; }
  ids_type              & dependencies    () { return dependencies_; }
  ids_type              & unknowns        () { return unknowns_;     }

  bool is_role_supported(std::string const& key, viennamini::role::segment_role_ids segment_role) 
  { 
    if(role_lookup_.find(key) == role_lookup_.end())
      throw viennamini::pde_set_exception("Quantity key \""+key+"\" is missing information regarding its support for specific segment-roles!");
    return role_lookup_[key].find(segment_role) != role_lookup_[key].end();
  }

  void register_quantity(std::string const& quantity_name, std::size_t quantity_id)
  {
    quantity_name_id_[quantity_name] = quantity_id;
  }

  void set_initial_guess(std::string const& quantity_name, viennamini::quantity_generator* init_guess)
  {
    initial_guess_lookup_[quantity_name] = init_guess;
  }

  viennamini::quantity_generator* get_initial_guess(std::string const& quantity_name, viennamini::device_handle& device_handle, std::size_t segment_index)
  {
    if(initial_guess_lookup_.find(quantity_name) == initial_guess_lookup_.end())
      throw viennamini::pde_set_exception("Initial guess \""+quantity_name+"\" is missing!");
    if(!initial_guess_lookup_[quantity_name])
      throw viennamini::pde_set_exception("Initial guess \""+quantity_name+"\" is not initialized!");
    initial_guess_lookup_[quantity_name]->set_device(device_handle.get());
    initial_guess_lookup_[quantity_name]->set_segment_index(segment_index);
    return initial_guess_lookup_[quantity_name];
  }

  void set_contact_model(std::string const& quantity_name, viennamini::contact_model* model)
  {
    contact_model_lookup_[quantity_name] = model;
  }

  viennamini::contact_model* get_contact_model(std::string const& quantity_name)
  {
    if(!this->has_contact_model(quantity_name))
      throw viennamini::pde_set_exception("Contact model not available for quantity \""+quantity_name+"\"!");
    contact_model_lookup_[quantity_name]->set_quantity_name(quantity_name);
    return contact_model_lookup_[quantity_name];
  }

  bool has_contact_model(std::string const& quantity_name)
  {
    if( contact_model_lookup_.find(quantity_name) != contact_model_lookup_.end() )
    {
      if(contact_model_lookup_[quantity_name]) return true;
      else return false;
    }
    else return false;
  }

protected:
  void add_dependency  (std::string dependency) { dependencies_.push_back(dependency); }
  void add_unknown     (std::string unknown)    { unknowns_.push_back(unknown); }

  void add_role_support(std::string key, viennamini::role::segment_role_ids segment_role)    
  { 
    role_lookup_[key].insert(segment_role);
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

  std::map<std::string, std::set<viennamini::role::segment_role_ids> >  role_lookup_;
  std::map<std::string, std::size_t>                                    quantity_name_id_;
  std::map<std::string, viennamini::quantity_generator*>                initial_guess_lookup_;
  std::map<std::string, viennamini::contact_model*>                     contact_model_lookup_;
};

} // viennamini


#endif


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

#include "viennamini/pde_set.hpp"

namespace viennamini
{

pde_set::pde_set()
{
}

pde_set::~pde_set()
{
  for(InitialGuessLookupType::iterator iter = initial_guess_lookup_.begin();
      iter != initial_guess_lookup_.end(); iter++)
  {
    if(iter->second) delete iter->second;
  }
}

viennamath::equation& pde_set::equation() 
{ 
  return equation_; 
}

pde_set::ids_type& pde_set::dependencies() 
{ 
  return dependencies_; 
}

pde_set::ids_type& pde_set::unknowns() 
{ 
  return unknowns_;     
}

bool pde_set::is_role_supported(std::string const& key, viennamini::role::segment_role_ids segment_role) 
{ 
  if(role_lookup_.find(key) == role_lookup_.end())
    throw viennamini::pde_set_exception("Quantity key \""+key+"\" is missing information regarding its support for specific segment-roles!");
  return role_lookup_[key].find(segment_role) != role_lookup_[key].end();
}

void pde_set::register_quantity(std::string const& quantity_name, std::size_t quantity_id)
{
  quantity_name_id_[quantity_name] = quantity_id;
}

void pde_set::set_initial_guess(std::string const& quantity_name, viennamini::quantity_generator* init_guess)
{
  initial_guess_lookup_[quantity_name] = init_guess;
}

viennamini::quantity_generator* pde_set::get_initial_guess(std::string const& quantity_name, viennamini::device_handle& device_handle, std::size_t segment_index)
{
  if(initial_guess_lookup_.find(quantity_name) == initial_guess_lookup_.end())
    throw viennamini::pde_set_exception("Initial guess \""+quantity_name+"\" is missing!");
  if(!initial_guess_lookup_[quantity_name])
    throw viennamini::pde_set_exception("Initial guess \""+quantity_name+"\" is not initialized!");
  initial_guess_lookup_[quantity_name]->set_device(device_handle.get());
  initial_guess_lookup_[quantity_name]->set_segment_index(segment_index);
  return initial_guess_lookup_[quantity_name];
}

void pde_set::set_contact_model(std::string const& quantity_name, viennamini::contact_model* model)
{
  contact_model_lookup_[quantity_name] = model;
}

viennamini::contact_model* pde_set::get_contact_model(std::string const& quantity_name)
{
  if(!this->has_contact_model(quantity_name))
    throw viennamini::pde_set_exception("Contact model not available for quantity \""+quantity_name+"\"!");
  contact_model_lookup_[quantity_name]->set_quantity_name(quantity_name);
  return contact_model_lookup_[quantity_name];
}

bool pde_set::has_contact_model(std::string const& quantity_name)
{
  if( contact_model_lookup_.find(quantity_name) != contact_model_lookup_.end() )
  {
    if(contact_model_lookup_[quantity_name]) return true;
    else return false;
  }
  else return false;
}

void pde_set::add_dependency  (std::string dependency) 
{ 
  dependencies_.push_back(dependency); 
}

void pde_set::add_unknown     (std::string unknown)    
{ 
  unknowns_.push_back(unknown); 
}

void pde_set::add_role_support(std::string key, viennamini::role::segment_role_ids segment_role)    
{ 
  role_lookup_[key].insert(segment_role);
}

std::size_t pde_set::get_quantity_id(std::string const& quantity_name) 
{ 
  if(quantity_name_id_.find(quantity_name) == quantity_name_id_.end())
    throw viennamini::pde_set_exception("Quantity \""+quantity_name+"\" has not been registered!");
  return quantity_name_id_[quantity_name];
}

} // viennamini


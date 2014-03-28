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

  pde_set();
  virtual ~pde_set();

  virtual std::string info()    = 0;
  virtual pdes_type get_pdes()  = 0;
  virtual bool is_linear()      = 0;

  viennamath::equation  & equation        ();
  ids_type              & dependencies    ();
  ids_type              & unknowns        ();

  bool is_role_supported(std::string const& key, viennamini::role::segment_role_ids segment_role);

  void register_quantity(std::string const& quantity_name, std::size_t quantity_id);

  void set_initial_guess(std::string const& quantity_name, viennamini::quantity_generator* init_guess);

  viennamini::quantity_generator* get_initial_guess(std::string const& quantity_name, viennamini::device_handle& device_handle, std::size_t segment_index);

  void set_contact_model(std::string const& quantity_name, viennamini::contact_model* model);

  viennamini::contact_model* get_contact_model(std::string const& quantity_name);

  bool has_contact_model(std::string const& quantity_name);

protected:
  void add_dependency  (std::string dependency);
  void add_unknown     (std::string unknown);
  void add_role_support(std::string key, viennamini::role::segment_role_ids segment_role);
  std::size_t get_quantity_id(std::string const& quantity_name);

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


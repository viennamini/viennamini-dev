#ifndef VIENNAMINI_DEVICESEGMENTPARAMETERS_HPP
#define VIENNAMINI_DEVICESEGMENTPARAMETERS_HPP

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

template<typename NumericT>
struct segment_parameters
{
public:
  typedef NumericT      NumericType;
  typedef NumericT      numeric_type;

  segment_parameters()
  {
    is_contact_       = false;
    is_oxide_         = false;
    is_semiconductor_ = false;
    name_             = "";
    material_         = "";
    contact_          = 0.0;
    workfunction_     = 0.0;
    NA_max_           = 0.0;
    ND_max_           = 0.0;
  }

  inline std::string& name()          { return name_; }
  inline std::string& material()      { return material_; }
  
  inline bool& is_contact()           { return is_contact_; }
  inline bool& is_oxide()             { return is_oxide_; }
  inline bool& is_semiconductor()     { return is_semiconductor_; }

  inline numeric_type& contact_potential()  { return contact_; }
  inline numeric_type& workfunction()       { return workfunction_; }
  inline numeric_type& NA_max()             { return NA_max_; }
  inline numeric_type& ND_max()             { return ND_max_; }  

private:
  std::string name_;
  std::string material_;

  bool is_contact_;
  bool is_oxide_;
  bool is_semiconductor_;

  numeric_type contact_;
  numeric_type workfunction_;
  numeric_type NA_max_;
  numeric_type ND_max_;
};

} // viennamini

#endif 


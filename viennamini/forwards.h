#ifndef VIENNAMINI_FORWARDS_H
#define VIENNAMINI_FORWARDS_H

/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the ViennaFVM base directory
======================================================================= */

/** Forward declarations */


//#include "viennamini/device.hpp"

#include <iostream>
#include <map>
#include <memory>

// ViennaGrid includes
#include "viennagrid/forwards.hpp"
#include "viennagrid/config/default_configs.hpp"
#include "viennagrid/mesh/segmented_mesh.hpp"

#include "viennamaterials/library.hpp"

namespace viennamini
{

  namespace id {
    inline std::string permittivity()               { return "Permittivity"; }
    inline std::string donor_doping()               { return "Donor Doping"; }
    inline std::string acceptor_doping()            { return "Acceptor Doping"; }
    inline std::string potential()                  { return "Potential"; }
    inline std::string electron_concentration()     { return "Electron Concentration"; }
    inline std::string hole_concentration()         { return "Hole Concentration"; }
    inline std::string electron_mobility()          { return "Electron Mobility"; }
    inline std::string hole_mobility()              { return "Hole Mobility"; }
    inline std::string curren_density()             { return "Current Density"; }
    inline std::string electric_field()             { return "Electric Field"; }
    inline std::string recombination()              { return "Recombination"; }
    inline std::string intrinsic_carrier()          { return "Intrinsic Carrier"; }
    inline std::string temperature()                { return "Temperature";  }
    inline std::string thermal_potential()          { return "Thermal Potential";  }
    inline std::string tau_n()                      { return "Electron minority lifetime"; }
    inline std::string tau_p()                      { return "Hole minority lifetime"; }
    inline std::string srh_n1()                     { return "SRH n1"; }
    inline std::string srh_p1()                     { return "SRH p1";  }
    inline std::string oxide()                      { return "oxide"; }
    inline std::string semiconductor()              { return "semiconductor"; }
    inline std::string contact()                    { return "contact"; }
  } // id

  // the XML Tags used in the materials file
  namespace material {
    inline std::string relative_permittivity()              { return "relative_permittivity"; }
    inline std::string intrinsic_carrier_concentration()    { return "intrinsic_carrier_concentration"; }
    inline std::string base_electron_mobility()             { return "mu_n_0"; }
    inline std::string base_hole_mobility()                 { return "mu_p_0"; }
    inline std::string alpha_n()                            { return "alpha_n"; }
    inline std::string alpha_p()                            { return "alpha_p"; }
    inline std::string tau_n_0()                            { return "tau_n_0"; }
    inline std::string tau_p_0()                            { return "tau_p_0"; }
    inline std::string mu_min_n()                           { return "mu_min_n"; }
    inline std::string mu_min_p()                           { return "mu_min_p"; }
    inline std::string N_ref_n()                            { return "N_ref_n"; }
    inline std::string N_ref_p()                            { return "N_ref_p"; }
    inline std::string value()                              { return "value"; }
    inline std::string unit()                               { return "unit"; }
    inline std::string drift_diffusion()                    { return "drift_diffusion"; }
    inline std::string lattice_scattering()                 { return "lattice_scattering"; }
    inline std::string ionized_impurity_scattering()        { return "ionized_impurity_scattering"; }
    inline std::string shockley_read_hall_recombination()   { return "shockley_read_hall_recombination"; }
    inline std::string oxide()                              { return "oxide"; }
    inline std::string name()                               { return "name"; }
    inline std::string id()                                 { return "id"; }
    inline std::string semiconductor()                      { return "semiconductor"; }
    inline std::string metal()                              { return "metal"; }
  } // material

  class line_1d         {};
  class triangular_2d   {};
  class tetrahedral_3d  {};



  namespace role {
    enum segment_role_ids
    {
      none,
      contact,
      oxide,
      semiconductor
    };
  } // role

  namespace recombination {
    enum recombination_ids
    {
      none,
      srh
    };
  } // recombination

  namespace mobility {
    enum mobility_ids
    {
      none,
      lattice,
      ionized_impurity
    };
  } // mobility

  namespace mesh {
    enum mesh_ids
    {
      none,
      line_1d, 
      triangular_2d, 
      tetrahedral_3d
    };
  } // mesh

  namespace pdeset {
    enum pdeset_ids
    {
      none,
      laplace, 
      drift_diffusion 
    };
  } // pdeset

  namespace discret {
    enum discret_ids
    {
      none,
      fvm, 
      fem
    };
  } // discret

  // public typedefs
  //

  class device;
  class configuration;
  class device_template;
  class simulator;
  class problem;
  class csv;
  class discretization;
  class stepper;
  class pde_set;

  typedef double                                                                                                  numeric;
  typedef std::map<std::size_t, numeric>                                                                          sparse_values;

  typedef ::viennagrid::mesh< viennagrid::config::line_1d >                                                       mesh_line_1d;
  typedef ::viennagrid::mesh< viennagrid::config::triangular_2d >                                                 mesh_triangular_2d;
  typedef ::viennagrid::mesh< viennagrid::config::tetrahedral_3d >                                                mesh_tetrahedral_3d;

  typedef ::viennagrid::result_of::segmentation<mesh_line_1d>::type                                               segmentation_line_1d;
  typedef ::viennagrid::result_of::segmentation<mesh_triangular_2d>::type                                         segmentation_triangular_2d;
  typedef ::viennagrid::result_of::segmentation<mesh_tetrahedral_3d>::type                                        segmentation_tetrahedral_3d;

  typedef segmentation_line_1d::segment_handle_type                                                               segment_line_1d;
  typedef segmentation_triangular_2d::segment_handle_type                                                         segment_triangular_2d;
  typedef segmentation_tetrahedral_3d::segment_handle_type                                                        segment_tetrahedral_3d;


  typedef ::viennagrid::segmented_mesh<viennamini::mesh_line_1d,  viennamini::segmentation_line_1d>               segmesh_line_1d;
  typedef ::viennagrid::segmented_mesh<viennamini::mesh_triangular_2d,  viennamini::segmentation_triangular_2d>   segmesh_triangular_2d;
  typedef ::viennagrid::segmented_mesh<viennamini::mesh_tetrahedral_3d, viennamini::segmentation_tetrahedral_3d>  segmesh_tetrahedral_3d;

#ifdef VIENNAMINI_WITH_CXX11
  typedef std::unique_ptr<viennamini::device>                                                                 device_handle;
  typedef std::unique_ptr<viennamini::configuration>                                                          configuration_handle;
  typedef std::unique_ptr<viennamaterials::library>                                                           material_library_handle;
  typedef std::unique_ptr<viennamini::device_template>                                                        device_template_handle;
  typedef std::unique_ptr<viennamini::simulator>                                                              simulator_handle;
  typedef std::unique_ptr<viennamini::stepper>                                                                stepper_handle;
  typedef std::unique_ptr<viennamini::discretization>                                                         discretization_handle;
  typedef std::unique_ptr<viennamini::pde_set>                                                                pde_set_handle;
#else
  typedef std::auto_ptr<viennamini::device>                                                                 device_handle;
  typedef std::auto_ptr<viennamini::configuration>                                                          configuration_handle;
  typedef std::auto_ptr<viennamaterials::library>                                                           material_library_handle;
  typedef std::auto_ptr<viennamini::device_template>                                                        device_template_handle;
  typedef std::auto_ptr<viennamini::simulator>                                                              simulator_handle;
  typedef std::auto_ptr<viennamini::stepper>                                                                stepper_handle;
  typedef std::auto_ptr<viennamini::discretization>                                                         discretization_handle;
  typedef std::auto_ptr<viennamini::pde_set>                                                                pde_set_handle;
#endif

} // viennamini

#endif

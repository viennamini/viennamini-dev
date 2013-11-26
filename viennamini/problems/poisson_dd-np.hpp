#ifndef VIENNAMINI_PROBLEMS_POISSON_DD_NP_HPP
#define VIENNAMINI_PROBLEMS_POISSON_DD_NP_HPP

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

#include "viennamini/problem.hpp"

namespace viennamini {

struct problem_poisson_dd_np : public problem
{
  VIENNAMINI_PROBLEM
  
  template<typename SegmentedMeshT, typename ProblemDescriptionT>
  void run_impl(SegmentedMeshT& segmesh, ProblemDescriptionT& problem_description)
  {
    namespace vmat = viennamaterials;

    typedef typename SegmentedMeshT::mesh_type                MeshType;
    typedef typename SegmentedMeshT::segmentation_type        SegmentationType;
    typedef typename ProblemDescriptionT::quantity_type       QuantityType;

    // -------------------------------------------------------------------------
    //
    // Extract ViennaFVM::Quantities
    //
    // -------------------------------------------------------------------------
    
    // access the already available quantities in the device's problem description
    //
    QuantityType & permittivity      = problem_description.get_quantity(viennamini::id::permittivity());
    QuantityType & potential         = problem_description.get_quantity(viennamini::id::potential());
    QuantityType & donator_doping    = problem_description.get_quantity(viennamini::id::donator_doping());
    QuantityType & acceptor_doping   = problem_description.get_quantity(viennamini::id::acceptor_doping());

    // add new quantities required for the simulation
    //
    QuantityType & electron_density         = problem_description.add_quantity(viennamini::id::electron_density());
    QuantityType & hole_density             = problem_description.add_quantity(viennamini::id::hole_density());
    QuantityType & electron_mobility        = problem_description.add_quantity(viennamini::id::electron_mobility());
    QuantityType & hole_mobility            = problem_description.add_quantity(viennamini::id::hole_mobility());
    QuantityType & recombination            = problem_description.add_quantity(viennamini::id::recombination());
    QuantityType & intrinsic_concentration  = problem_description.add_quantity(viennamini::id::intrinsic_carrier());
    QuantityType & temperature              = problem_description.add_quantity(viennamini::id::temperature());
    QuantityType & thermal_pot              = problem_description.add_quantity(viennamini::id::thermal_potential());

    QuantityType & electron_lifetime        = problem_description.add_quantity(viennamini::id::tau_n());
    QuantityType & hole_lifetime            = problem_description.add_quantity(viennamini::id::tau_p());
    QuantityType & srh_n1                   = problem_description.add_quantity(viennamini::id::srh_n1());
    QuantityType & srh_p1                   = problem_description.add_quantity(viennamini::id::srh_p1());

    // -------------------------------------------------------------------------
    //
    // Assign segment roles: setup initial guesses and boundary conditions
    //
    // -------------------------------------------------------------------------

    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin(); 
        sit != segmesh.segmentation.end(); ++sit)
    {
      std::size_t current_segment_index = sit->id();

      std::string material = device().get_material(current_segment_index);
      std::string name     = device().get_name(current_segment_index);

    #ifdef VIENNAMINI_VERBOSE
      std::cout << std::endl;
      std::cout << "[Problem][PoissonDD NP] Processing segment " << current_segment_index << std::endl;
      std::cout << "  Name:     \"" << name << "\"" << std::endl;
      std::cout << "  Material: \"" << material << "\"" << std::endl;
    #endif
      
      //
      // Set quantities on all segments 
      //
      // temperature
      viennafvm::set_initial_value(temperature, segmesh.segmentation(current_segment_index), config().temperature()); 

      // thermal potential
      viennafvm::set_initial_value(thermal_pot, segmesh.segmentation(current_segment_index), thermal_potential<QuantityType>(temperature)); 
      

      if(device().is_contact(current_segment_index))
      {
        if(device().is_contact_at_semiconductor(current_segment_index))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  identified as a contact next to a semiconductor .." << std::endl;
        #endif
          std::size_t adjacent_semiconductor_segment_index = device().get_adjacent_semiconductor_segment_for_contact(current_segment_index);
          NumericType ND_value    = device().get_donator_doping(adjacent_semiconductor_segment_index);
          NumericType NA_value    = device().get_acceptor_doping(adjacent_semiconductor_segment_index);

          vmat::query myquery = vmat::make_query(
            vmat::make_entry(device().matlib_material() , device().get_material(adjacent_semiconductor_segment_index)), 
            vmat::make_entry(device().matlib_parameter(), material::intrinsic_carrier_concentration()),
            vmat::make_entry(device().matlib_data()     , material::value()));


          NumericType ni_value    = device().material_library()->query_value(myquery);
          NumericType builtin_pot = viennamini::built_in_potential_impl(ND_value, NA_value, config().temperature(), ni_value);
        
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  ND:      " << ND_value << std::endl;
          std::cout << "  NA:      " << NA_value << std::endl;
          std::cout << "  ni:      " << ni_value << std::endl;
          std::cout << "  builtin: " << builtin_pot << std::endl;
        #endif

          // add the builtin potential to the dirichlet potential boundary
          viennafvm::addto_dirichlet_boundary(potential, 
                                            segmesh.segmentation(current_segment_index), 
                                            builtin_pot);
        
          // electrons dirichlet boundary
          viennafvm::set_dirichlet_boundary(electron_density, segmesh.segmentation(current_segment_index), ND_value);

          // holes dirichlet boundary
          viennafvm::set_dirichlet_boundary(hole_density, segmesh.segmentation(current_segment_index), NA_value);
        }
        else
        if(device().is_contact_at_oxide(current_segment_index))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  identified as a contact next to an oxide .." << std::endl;
        #endif
        }
        else throw segment_undefined_contact_exception(current_segment_index);
      }
      else
      if(device().is_oxide(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "  identified as an oxide .." << std::endl;
      #endif
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      else
      if(device().is_semiconductor(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "  identified as a semiconductor .." << std::endl;
      #endif
      
        NumericType ni_value    = device().material_library()->query_value(
          vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                           vmat::make_entry(device().matlib_parameter(), material::intrinsic_carrier_concentration()),
                           vmat::make_entry(device().matlib_data()     , material::value()))
        );

        // intrinsic carrier concentration
        viennafvm::set_initial_value(intrinsic_concentration, segmesh.segmentation(current_segment_index), ni_value); 

        // potential
        viennafvm::set_initial_value(potential, segmesh.segmentation(current_segment_index), built_in_potential<QuantityType>(donator_doping, acceptor_doping, intrinsic_concentration, temperature)); 
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));

        // electrons
        viennafvm::set_initial_value(electron_density, segmesh.segmentation(current_segment_index), donator_doping);
        viennafvm::set_unknown(electron_density, segmesh.segmentation(current_segment_index));

        // holes
        viennafvm::set_initial_value(hole_density, segmesh.segmentation(current_segment_index), acceptor_doping);
        viennafvm::set_unknown(hole_density, segmesh.segmentation(current_segment_index));

        // mobility
        NumericType mu_n_0_value    = device().material_library()->query_value(
          vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                           vmat::make_entry(device().matlib_parameter(), material::base_electron_mobility()),
                           vmat::make_entry(device().matlib_data()     , material::value()))
        );
        NumericType mu_p_0_value    = device().material_library()->query_value(
          vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                           vmat::make_entry(device().matlib_parameter(), material::base_hole_mobility()),
                           vmat::make_entry(device().matlib_data()     , material::value()))
        );
        if(device().get_mobility(current_segment_index) == mobility::base)
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    using base mobilities: " << std::endl;
        #endif
          viennafvm::set_initial_value(electron_mobility, segmesh.segmentation(current_segment_index),  mu_n_0_value); 
          viennafvm::set_initial_value(hole_mobility, segmesh.segmentation(current_segment_index),      mu_p_0_value); 
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "      mu n 0: " << mu_n_0_value << " mu p 0: " << mu_p_0_value << std::endl;
        #endif
        }
        else
        if(device().get_mobility(current_segment_index) == mobility::lattice)
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    activating lattice scattering mobility model .." << std::endl;
        #endif
          NumericType alpha_n_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::lattice_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_n()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType alpha_p_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::lattice_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_p()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "      mu n 0: " << mu_n_0_value << " mu p 0: " << mu_p_0_value << std::endl;
          std::cout << "      alpha n: " << alpha_n_value << " alpha p: " << alpha_p_value << std::endl;
        #endif
          viennafvm::set_initial_value(electron_mobility, segmesh.segmentation(current_segment_index), mobility::lattice_scattering<QuantityType>(mu_n_0_value, alpha_n_value, temperature)); 
          viennafvm::set_initial_value(hole_mobility,     segmesh.segmentation(current_segment_index), mobility::lattice_scattering<QuantityType>(mu_p_0_value, alpha_p_value, temperature)); 
        }
        else
        if(device().get_mobility(current_segment_index) == mobility::ionized_impurity)
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    activating ionized impurity scattering mobility model .." << std::endl;
        #endif
          NumericType alpha_n_lattice_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::lattice_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_n()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType alpha_p_lattice_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::lattice_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_p()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
        
          NumericType alpha_n_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_n()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType alpha_p_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::alpha_p()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType mu_min_n_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::mu_min_n()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType mu_min_p_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::mu_min_p()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType N_ref_n_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::N_ref_n()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
          NumericType N_ref_p_value    = device().material_library()->query_value(
            vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                             vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                             vmat::make_entry(device().matlib_model(),     material::ionized_impurity_scattering()),
                             vmat::make_entry(device().matlib_parameter(), material::N_ref_p()),
                             vmat::make_entry(device().matlib_data()     , material::value()))
          );
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "      alpha n: " << alpha_n_value << " alpha p: " << alpha_p_value << std::endl;
          std::cout << "      mu min n: " << mu_min_n_value << " mu min p: " << mu_min_p_value << std::endl;
          std::cout << "      N ref n: " << N_ref_n_value << " N ref p: " << N_ref_p_value << std::endl;
        #endif
          
          typedef mobility::lattice_scattering<QuantityType>                            LatticeType;
          LatticeType lattice_n(mu_n_0_value, alpha_n_lattice_value, temperature);
          LatticeType lattice_p(mu_p_0_value, alpha_p_lattice_value, temperature);
          
          typedef mobility::ionized_impurity_scattering<LatticeType, QuantityType>      IonizedImpurityType;
          IonizedImpurityType   ionized_impurities_n(lattice_n, donator_doping, acceptor_doping, alpha_n_value, mu_min_n_value, N_ref_n_value);
          IonizedImpurityType   ionized_impurities_p(lattice_p, donator_doping, acceptor_doping, alpha_p_value, mu_min_p_value, N_ref_p_value);
          
          viennafvm::set_initial_value(electron_mobility, segmesh.segmentation(current_segment_index), ionized_impurities_n); 
          viennafvm::set_initial_value(hole_mobility,     segmesh.segmentation(current_segment_index), ionized_impurities_p); 
        }
        else throw mobility_not_supported_exception();

        // recombination
        if(device().get_recombination(current_segment_index) == recombination::none)
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    deactivating recombination models .." << std::endl;
        #endif
          viennafvm::set_initial_value(recombination,     segmesh.segmentation(current_segment_index), 0.0); // switch off
          
          viennafvm::set_initial_value(electron_lifetime, segmesh.segmentation(current_segment_index), 0.0);
          viennafvm::set_initial_value(hole_lifetime,     segmesh.segmentation(current_segment_index), 0.0);
          viennafvm::set_initial_value(srh_n1,            segmesh.segmentation(current_segment_index), 0.0); 
          viennafvm::set_initial_value(srh_p1,            segmesh.segmentation(current_segment_index), 0.0); 
        }
        else
        if(device().get_recombination(current_segment_index) == recombination::srh)
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    activating SRH recombination model .." << std::endl;
        #endif
          viennafvm::set_initial_value(recombination, segmesh.segmentation(current_segment_index), 1.0); // switch

          NumericType tau_n_0 = device().material_library()->query_value(
              vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                               vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                               vmat::make_entry(device().matlib_model(),     material::shockley_read_hall_recombination()),
                               vmat::make_entry(device().matlib_parameter(), material::tau_n_0()),
                               vmat::make_entry(device().matlib_data()     , material::value()))  );

          NumericType tau_p_0 = device().material_library()->query_value(
              vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                               vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                               vmat::make_entry(device().matlib_model(),     material::shockley_read_hall_recombination()),
                               vmat::make_entry(device().matlib_parameter(), material::tau_p_0()),
                               vmat::make_entry(device().matlib_data()     , material::value()))  );

          NumericType N_ref_n = device().material_library()->query_value(
              vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                               vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                               vmat::make_entry(device().matlib_model(),     material::shockley_read_hall_recombination()),
                               vmat::make_entry(device().matlib_parameter(), material::N_ref_n()),
                               vmat::make_entry(device().matlib_data()     , material::value()))  );

          NumericType N_ref_p = device().material_library()->query_value(
              vmat::make_query(vmat::make_entry(device().matlib_material() , material), 
                               vmat::make_entry(device().matlib_model(),     material::drift_diffusion()),
                               vmat::make_entry(device().matlib_model(),     material::shockley_read_hall_recombination()),
                               vmat::make_entry(device().matlib_parameter(), material::N_ref_p()),
                               vmat::make_entry(device().matlib_data()     , material::value()))  );

          carrier_lifetimes<QuantityType> tau_n(donator_doping, acceptor_doping, N_ref_n, tau_n_0);
          carrier_lifetimes<QuantityType> tau_p(donator_doping, acceptor_doping, N_ref_p, tau_p_0);

          viennafvm::set_initial_value(electron_lifetime, segmesh.segmentation(current_segment_index), tau_n);
          viennafvm::set_initial_value(hole_lifetime,     segmesh.segmentation(current_segment_index), tau_p);
          
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "      tau n 0: " << tau_n_0 << " tau p 0: " << tau_p_0 << std::endl;
          std::cout << "      N ref n: " << N_ref_n << " N ref p: " << N_ref_p << std::endl;
        #endif
          viennafvm::set_initial_value(srh_n1,                segmesh.segmentation(current_segment_index), electron_density); 
          viennafvm::set_initial_value(srh_p1,                segmesh.segmentation(current_segment_index), hole_density); 
        }
        else throw recombination_not_supported_exception();
      }
      else throw segment_undefined_exception(current_segment_index);
    }

    // -------------------------------------------------------------------------
    //
    // Specify partial differential equations
    //
    // -------------------------------------------------------------------------

    FunctionSymbolType psi        (potential.id());
    FunctionSymbolType n          (electron_density.id());
    FunctionSymbolType p          (hole_density.id());
    FunctionSymbolType mu_n       (electron_mobility.id());
    FunctionSymbolType mu_p       (hole_mobility.id());
    FunctionSymbolType VT         (thermal_pot.id());
    FunctionSymbolType epsr       (permittivity.id());
    FunctionSymbolType ND         (donator_doping.id());
    FunctionSymbolType NA         (acceptor_doping.id());
    FunctionSymbolType ni         (intrinsic_concentration.id());
    FunctionSymbolType R_switch   (recombination.id());   // is either 0 or 1, allows segment-wise activation of recombination

    FunctionSymbolType tau_n      (electron_lifetime.id());
    FunctionSymbolType tau_p      (hole_lifetime.id());
    FunctionSymbolType n1         (srh_n1.id());
    FunctionSymbolType p1         (srh_p1.id());

    NumericType q          = viennamini::q::val();
//    viennamath::expr R_srh_left  = R_switch *      (n  * p ) / (tau_p*(n + n1) + tau_n*(p + p1));
//    viennamath::expr R_srh_right = R_switch * -1 * (ni * ni) / (tau_p*(n + n1) + tau_n*(p + p1));
    viennamath::expr R_srh_left  = R_switch *      (n  * p ) / (1.e-5*(n + n1)+tau_n*(p + p1));
    viennamath::expr R_srh_right = R_switch * -1 * (ni * ni) / (tau_p*(n + n1)+1.e-5*(p + p1));

    EquationType poisson_eq = viennamath::make_equation( viennamath::div(epsr * viennamath::grad(psi)),                                                    /* = */ q * ((n - ND) - (p - NA)));

    EquationType cont_eq_n  = viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(n) - mu_n * viennamath::grad(psi) * n) - R_srh_left, /* = */ R_srh_right);
    EquationType cont_eq_p  = viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(p) + mu_p * viennamath::grad(psi) * p) - R_srh_left, /* = */ R_srh_right);

    // Specify the PDE system:
    viennafvm::linear_pde_system<> pde_system;
    pde_system.add_pde(poisson_eq, psi); pde_system.option(0).damping_term( (n + p) * (-q / VT) );
    pde_system.add_pde(cont_eq_n,  n);   pde_system.option(1).geometric_update(true);
    pde_system.add_pde(cont_eq_p,  p);   pde_system.option(2).geometric_update(true);

    pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running

    // -------------------------------------------------------------------------
    //
    // Assemble and solve the problem
    //
    // -------------------------------------------------------------------------
    viennafvm::linsolv::viennacl  linear_solver;
    linear_solver.break_tolerance() = config().linear_breaktol();
    linear_solver.max_iterations()  = config().linear_iterations();
    
    viennafvm::pde_solver pde_solver;
    pde_solver.set_nonlinear_iterations(config().nonlinear_iterations());
    pde_solver.set_nonlinear_breaktol(config().nonlinear_breaktol());
    pde_solver.set_damping(config().damping());

    if(config().write_initial_guesses())
      this->write("initial");

  #ifdef VIENNAMINI_VERBOSE
    std::cout << std::endl;
    std::cout << "[Problem][PoissonDD NP] solving .. " << std::endl;
    std::cout << std::endl;
  #endif

    pde_solver(problem_description, pde_system, linear_solver);
  }
};

} // viennamini

#endif


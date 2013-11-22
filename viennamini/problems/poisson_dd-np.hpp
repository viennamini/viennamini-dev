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
public:
  problem_poisson_dd_np(viennamini::device& device, viennamini::config& config) : problem (device, config) { }

  void run()
  {
    if(device_.is_line1d())
    {
      this->run_impl(device_.get_segmesh_line_1d(), device_.get_problem_description_line_1d());
    }
    else
    if(device_.is_triangular2d())
    {
      this->run_impl(device_.get_segmesh_triangular_2d(), device_.get_problem_description_triangular_2d());
    }
    else 
    if(device_.is_tetrahedral3d())
    {
      this->run_impl(device_.get_segmesh_tetrahedral_3d(), device_.get_problem_description_tetrahedral_3d());
    }
    else throw device_not_supported_exception("at: problem_poisson_dd_np::run()");
  }
  
  
private:
  
  template<typename SegmentedMeshT, typename ProblemDescriptionT>
  void run_impl(SegmentedMeshT& segmesh, ProblemDescriptionT& problem_description)
  {
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
    QuantityType & electron_density  = problem_description.add_quantity(viennamini::id::electron_density());
    QuantityType & hole_density      = problem_description.add_quantity(viennamini::id::hole_density());
    QuantityType & electron_mobility = problem_description.add_quantity(viennamini::id::electron_mobility());
    QuantityType & hole_mobility     = problem_description.add_quantity(viennamini::id::hole_mobility());

    // -------------------------------------------------------------------------
    //
    // Assign segment roles: setup initial guesses and boundary conditions
    //
    // -------------------------------------------------------------------------

    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin(); 
        sit != segmesh.segmentation.end(); ++sit)
    {
      std::size_t current_segment_index = sit->id();
      
      // TODO the following Contact-Semiconductor handling needs to be outsourced to ViennaFVM
      if(device_.is_contact(current_segment_index))
      {
        if(device_.is_contact_at_semiconductor(current_segment_index))
        {
          std::size_t adjacent_semiconductor_segment_index = device_.get_adjacent_semiconductor_segment_for_contact(current_segment_index);
          NumericType ND_value    = device_.get_donator_doping(adjacent_semiconductor_segment_index);
          NumericType NA_value    = device_.get_acceptor_doping(adjacent_semiconductor_segment_index);
          NumericType ni_value    = device_.material_library()()->get_parameter_value(
                                      device_.get_material(adjacent_semiconductor_segment_index), 
                                      viennamini::material::intrinsic_carrier_concentration());

          NumericType builtin_pot = viennamini::built_in_potential_impl(ND_value, NA_value, config_.temperature(), ni_value);
        
          // add the builtin potential to the dirichlet potential boundary
          viennafvm::addto_dirichlet_boundary(potential, 
                                            segmesh.segmentation(current_segment_index), 
                                            builtin_pot);
        
          // electrons dirichlet boundary
          viennafvm::set_dirichlet_boundary(electron_density, segmesh.segmentation(current_segment_index), ND_value);

          // holes dirichlet boundary
          viennafvm::set_dirichlet_boundary(hole_density, segmesh.segmentation(current_segment_index), NA_value);
        }
      }
      else
      if(device_.is_oxide(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "solving potential for oxide segment: " << current_segment_index << std::endl;
      #endif
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      else
      if(device_.is_semiconductor(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "solving potential for semiconductor segment: " << current_segment_index << std::endl;
      #endif
      
        NumericType ni_value    = device_.material_library()()->get_parameter_value(
          device_.get_material(current_segment_index), viennamini::material::intrinsic_carrier_concentration());

        // potential
        viennafvm::set_initial_value(potential, segmesh.segmentation(current_segment_index), built_in_potential<QuantityType>(donator_doping, acceptor_doping, ni_value, config_.temperature())); 
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));

        // electrons
        viennafvm::set_initial_value(electron_density, segmesh.segmentation(current_segment_index), donator_doping);
        viennafvm::set_unknown(electron_density, segmesh.segmentation(current_segment_index));

        // holes
        viennafvm::set_initial_value(hole_density, segmesh.segmentation(current_segment_index), acceptor_doping);
        viennafvm::set_unknown(hole_density, segmesh.segmentation(current_segment_index));
        
        // electrons mobility
        viennafvm::set_initial_value(electron_mobility, segmesh.segmentation(current_segment_index), 1.0); // TODO for the time being, use constant 

        // holes mobility
        viennafvm::set_initial_value(hole_mobility, segmesh.segmentation(current_segment_index), 1.0); // TODO for the time being, use constant 
      }
    }

    // -------------------------------------------------------------------------
    //
    // Specify partial differential equations
    //
    // -------------------------------------------------------------------------

    FunctionSymbolType psi  (potential.id());
    FunctionSymbolType n    (electron_density.id());
    FunctionSymbolType p    (hole_density.id());
    FunctionSymbolType mu_n (electron_mobility.id());
    FunctionSymbolType mu_p (hole_mobility.id());
    FunctionSymbolType epsr (permittivity.id());
    FunctionSymbolType ND   (donator_doping.id());
    FunctionSymbolType NA   (acceptor_doping.id());

    // TODO: outsource to constants.hpp and physics.hpp
    NumericType q       = viennamini::q::val();
    viennamath::expr VT = viennamini::get_thermal_potential(config_.temperature());

    EquationType poisson_eq = viennamath::make_equation( viennamath::div(epsr * viennamath::grad(psi)),                               /* = */ q * ((n - ND) - (p - NA)));
    EquationType cont_eq_n  = viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(n) - mu_n * viennamath::grad(psi) * n), /* = */ 0);
    EquationType cont_eq_p  = viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(p) + mu_p * viennamath::grad(psi) * p), /* = */ 0);

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
    linear_solver.break_tolerance() = config_.linear_breaktol();
    linear_solver.max_iterations()  = config_.linear_iterations();
    
    viennafvm::pde_solver pde_solver;
    pde_solver.set_nonlinear_iterations(config_.nonlinear_iterations());
    pde_solver.set_nonlinear_breaktol(config_.nonlinear_breaktol());
    pde_solver.set_damping(config_.damping());
    pde_solver(problem_description, pde_system, linear_solver);
  }
};

} // viennamini

#endif


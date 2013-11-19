#ifndef VIENNAMINI_PROBLEMPOISSON_DD_NP_HPP
#define VIENNAMINI_PROBLEMPOISSON_DD_NP_HPP

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
#include "viennamini/physics.hpp"

// ViennaFVM includes:
#include "viennafvm/pde_solver.hpp"
#include "viennafvm/problem_description.hpp"

namespace viennamini {

struct problem_poisson_dd_np : public problem
{
public:
  problem_poisson_dd_np(viennamini::device& device, viennamini::config& config, viennamini::material_library& matlib) :
    problem         (device, config, matlib)
  {
  }

  void run()
  {
    if(device_.is_triangular2d())
    {
      problem_description_ = problem_description_triangular_2d();
      this->run_impl(device_.get_segmesh_triangular_2d(), boost::get<problem_description_triangular_2d>(problem_description_));
    }
    else 
    if(device_.is_tetrahedral3d())
    {
      problem_description_ = problem_description_tetrahedral_3d();
      this->run_impl(device_.get_segmesh_tetrahedral_3d(), boost::get<problem_description_tetrahedral_3d>(problem_description_));
    }
    else
      std::cout << "detect_interfaces: segmented mesh type not supported" << std::endl;
  }
  
  
private:
  
  template<typename SegmentedMeshT, typename ProblemDescriptionT>
  void run_impl(SegmentedMeshT& segmesh, ProblemDescriptionT& problem_description)
  {
    typedef typename SegmentedMeshT::mesh_type                MeshType;
    typedef typename SegmentedMeshT::segmentation_type        SegmentationType;
    typedef typename ProblemDescriptionT::quantity_type       QuantityType;

    problem_description = segmesh.mesh;
    
    QuantityType & potential         = problem_description.add_quantity(viennamini::id::potential());
    QuantityType & electron_density  = problem_description.add_quantity(viennamini::id::electron_density());
    QuantityType & hole_density      = problem_description.add_quantity(viennamini::id::hole_density());
    QuantityType & electron_mobility = problem_description.add_quantity(viennamini::id::electron_mobility());
    QuantityType & hole_mobility     = problem_description.add_quantity(viennamini::id::hole_mobility());
    QuantityType & permittivity      = problem_description.add_quantity(viennamini::id::permittivity());
    QuantityType & donator_doping    = problem_description.add_quantity(viennamini::id::donator_doping());
    QuantityType & acceptor_doping   = problem_description.add_quantity(viennamini::id::acceptor_doping());
    
    // -------------------------------------------------------------------------
    //
    // Assign segment roles: setup initial guesses and boundary conditions
    //
    // -------------------------------------------------------------------------
    
    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin(); 
        sit != segmesh.segmentation.end(); ++sit)
    {
      std::size_t current_segment_index = sit->id();
      
      if(device_.is_contact(current_segment_index))
      {
        if(device_.is_contact_at_oxide(current_segment_index))
        {
          // permittivity
          std::size_t adjacent_oxide_segment_index = device_.get_adjacent_oxide_segment_for_contact(current_segment_index);
          NumericType epsr = 0.0;
          if(device_.is_manual(adjacent_oxide_segment_index))
            epsr = device_.epsr(adjacent_oxide_segment_index);
          else epsr = matlib_()->get_parameter_value(device_.material(adjacent_oxide_segment_index), viennamini::mat::permittivity());
          viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
            epsr * viennamini::eps0::val());

        #ifdef VIENNAMINI_DEBUG
          std::cout << "segment " << current_segment_index << " is a contact (-oxide) segment " << std::endl;
          std::cout << "  contact value: " << device_.contact_potential(current_segment_index) << " workfunction: " << device_.workfunction(current_segment_index) << std::endl;
          std::cout << "  @oxide neighbor: epsr: " << epsr << std::endl;
        #endif

        
          // potential dirichlet boundary
          viennafvm::set_dirichlet_boundary(potential, 
                                            segmesh.segmentation(current_segment_index), 
                                            device_.contact_potential(current_segment_index) + device_.workfunction(current_segment_index));
        }
        else
        if(device_.is_contact_at_semiconductor(current_segment_index))
        {
          std::size_t adjacent_semiconductor_segment_index = device_.get_adjacent_semiconductor_segment_for_contact(current_segment_index);
          NumericType ND_value    = device_.ND_max(adjacent_semiconductor_segment_index);
          NumericType NA_value    = device_.NA_max(adjacent_semiconductor_segment_index);
          NumericType builtin_pot = viennamini::built_in_potential(config_.temperature(), ND_value, NA_value);

          // permittivity
          NumericType epsr = 0.0;
          if(device_.is_manual(adjacent_semiconductor_segment_index))
            epsr = device_.epsr(adjacent_semiconductor_segment_index);
          else epsr = matlib_()->get_parameter_value(device_.material(adjacent_semiconductor_segment_index), viennamini::mat::permittivity());
          viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
            epsr * viennamini::eps0::val());

        #ifdef VIENNAMINI_DEBUG
          std::cout << "segment " << current_segment_index << " is a contact (-semiconductor) segment " << std::endl;
          std::cout << "  contact value: " << device_.contact_potential(current_segment_index) << " workfunction: " << device_.workfunction(current_segment_index) << std::endl;
          std::cout << "  @semiconductor neighbor: ND: " << ND_value << " NA: " << NA_value << " builtin: " << builtin_pot << " epsr: " << epsr << std::endl;
        #endif

          // potential dirichlet boundary
          viennafvm::set_dirichlet_boundary(potential, 
                                            segmesh.segmentation(current_segment_index), 
                                            device_.contact_potential(current_segment_index) + device_.workfunction(current_segment_index) + builtin_pot);

          // electrons dirichlet boundary
          viennafvm::set_dirichlet_boundary(electron_density, segmesh.segmentation(current_segment_index), ND_value);

          // holes dirichlet boundary
          viennafvm::set_dirichlet_boundary(hole_density, segmesh.segmentation(current_segment_index), NA_value);
        }
        else
        {
          std::cout << "Error: Contact segment " << current_segment_index << 
            " is neither sharing an interface with an oxide nor with a semiconductor" << std::endl;
          exit(-1);
        }
      }
      if(device_.is_oxide(current_segment_index))
      {
        // permittivity
        NumericType epsr = 0.0;
        if(device_.is_manual(current_segment_index))
          epsr = device_.epsr(current_segment_index);
        else epsr = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
          epsr * viennamini::eps0::val());

      #ifdef VIENNAMINI_DEBUG
        std::cout << "segment " << current_segment_index << " is an oxide segment " << std::endl;
        std::cout << "  epsr: " << epsr << std::endl;
      #endif
      
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      if(device_.is_semiconductor(current_segment_index))
      {
        NumericType ND_value    = device_.ND_max(current_segment_index);
        NumericType NA_value    = device_.NA_max(current_segment_index);
        NumericType builtin_pot = viennamini::built_in_potential(config_.temperature(), ND_value, NA_value);
        NumericType epsr = 0.0;
        if(device_.is_manual(current_segment_index))
          epsr = device_.epsr(current_segment_index);
        else epsr = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
      
      #ifdef VIENNAMINI_DEBUG
        std::cout << "segment " << current_segment_index << " is a semiconductor segment " << std::endl;
        std::cout << "  ND: " << ND_value << " NA: " << NA_value << " builtin: " << builtin_pot << " epsr " << epsr << std::endl;
      #endif
        // potential
        viennafvm::set_initial_value(potential, segmesh.segmentation(current_segment_index), builtin_pot);
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
        
        // permittivity
        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
          epsr * viennamini::eps0::val());

        // electrons
        viennafvm::set_initial_value(electron_density, segmesh.segmentation(current_segment_index), ND_value);
        viennafvm::set_unknown(electron_density, segmesh.segmentation(current_segment_index));

        // holes
        viennafvm::set_initial_value(hole_density, segmesh.segmentation(current_segment_index), NA_value);
        viennafvm::set_unknown(hole_density, segmesh.segmentation(current_segment_index));
        
        // electrons mobility
        viennafvm::set_initial_value(electron_mobility, segmesh.segmentation(current_segment_index), 1.0);

        // holes mobility
        viennafvm::set_initial_value(hole_mobility, segmesh.segmentation(current_segment_index), 1.0);
        
        // donator doping
        viennafvm::set_initial_value(donator_doping, segmesh.segmentation(current_segment_index), ND_value);

        // acceptor doping
        viennafvm::set_initial_value(acceptor_doping, segmesh.segmentation(current_segment_index), NA_value);
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

    // here is all the fun: specify DD system
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

  void write(std::string const& filename)
  {
    if(device_.is_triangular2d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        boost::get<problem_description_triangular_2d>(problem_description_).quantities(), 
        filename, 
        device_.get_segmesh_triangular_2d().mesh, 
        device_.get_segmesh_triangular_2d().segmentation);
    }
    else 
    if(device_.is_tetrahedral3d())
    {
      viennafvm::io::write_solution_to_VTK_file(
        boost::get<problem_description_tetrahedral_3d>(problem_description_).quantities(), 
        filename, 
        device_.get_segmesh_tetrahedral_3d().mesh, 
        device_.get_segmesh_tetrahedral_3d().segmentation);
    }
  }
};

} // viennamini

#endif


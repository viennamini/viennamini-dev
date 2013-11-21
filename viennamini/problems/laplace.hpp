#ifndef VIENNAMINI_PROBLEMS_LAPLACE_HPP
#define VIENNAMINI_PROBLEMS_LAPLACE_HPP

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

struct problem_laplace : public problem
{
public:
  problem_laplace(viennamini::device& device, viennamini::config& config) :
    problem         (device, config)
  {
  }

  void run()
  {
    if(device_.is_triangular2d())
    {
      this->run_impl(device_.get_segmesh_triangular_2d(), device_.get_problem_description_triangular_2d());
    }
    else 
    if(device_.is_tetrahedral3d())
    {
      this->run_impl(device_.get_segmesh_tetrahedral_3d(), device_.get_problem_description_tetrahedral_3d());
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
    

    // -------------------------------------------------------------------------
    //
    // Extract ViennaFVM::Quantities
    //
    // -------------------------------------------------------------------------
    
    QuantityType & permittivity      = problem_description.get_quantity(viennamini::id::permittivity());
    QuantityType & potential         = problem_description.get_quantity(viennamini::id::potential());
    
    // -------------------------------------------------------------------------
    //
    // Assign segment roles: setup initial guesses and boundary conditions
    //
    // -------------------------------------------------------------------------
    
    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin(); 
        sit != segmesh.segmentation.end(); ++sit)
    {
      std::size_t current_segment_index = sit->id();
      
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
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
    }

    // -------------------------------------------------------------------------
    //
    // Specify partial differential equations
    //
    // -------------------------------------------------------------------------

    FunctionSymbolType psi  (potential.id());
    FunctionSymbolType epsr (permittivity.id());

    EquationType laplace_eq = viennamath::make_equation( viennamath::div(epsr * viennamath::grad(psi)), /* = */ 0);

    // Specify the PDE system:
    viennafvm::linear_pde_system<> pde_system;
    pde_system.add_pde(laplace_eq, psi); 
    pde_system.is_linear(true); 

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


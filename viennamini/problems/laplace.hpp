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

namespace viennamini {

struct problem_laplace : public problem
{
public:
  problem_laplace(viennamini::device& device, viennamini::config& config) : problem (device, config) { }

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
    else throw device_not_supported_exception("at: problem_laplace::run()");
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

    #ifdef VIENNAMINI_VERBOSE
      std::cout << std::endl;
      std::cout << "[Problem][Laplace] Processing segment " << current_segment_index << std::endl;
      std::cout << "  Name:     \"" << device_.get_name(current_segment_index) << "\"" << std::endl;
      std::cout << "  Material: \"" << device_.get_material(current_segment_index) << "\"" << std::endl;
    #endif

      if(device_.is_contact(current_segment_index))
      {
        if(device_.is_contact_at_semiconductor(current_segment_index))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  identified as a contact next to a semiconductor .." << std::endl;
        #endif
        }
        else
        if(device_.is_contact_at_oxide(current_segment_index))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  identified as a contact next to an oxide .." << std::endl;
        #endif
        }
        else throw segment_undefined_contact_exception(current_segment_index);
      }
      else
      if(device_.is_oxide(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "  identified as an oxide .." << std::endl;
      #endif
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      else
      if(device_.is_semiconductor(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        std::cout << "  identified as a semiconductor .." << std::endl;
      #endif
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      else throw segment_undefined_exception(current_segment_index);
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

    if(config_.write_initial_guesses())
      this->write("initial");

  #ifdef VIENNAMINI_VERBOSE
    std::cout << std::endl;
    std::cout << "[Problem][Laplace] solving .. " << std::endl;
    std::cout << std::endl;
  #endif

    pde_solver(problem_description, pde_system, linear_solver);
  }
};

} // viennamini

#endif


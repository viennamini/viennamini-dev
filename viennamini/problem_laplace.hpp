#ifndef VIENNAMINI_PROBLEM_LAPLACE_HPP
#define VIENNAMINI_PROBLEM_LAPLACE_HPP

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
  problem_laplace(viennamini::device& device, viennamini::config& config, viennamini::material_library& matlib) :
    problem         (device, config, matlib),
    psi_            (0),
    permittivity_   (1)
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
    QuantityType & permittivity      = problem_description.add_quantity(viennamini::id::permittivity());
    
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
          NumericType epsr = matlib_()->get_parameter_value(device_.material(adjacent_oxide_segment_index), viennamini::mat::permittivity());
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

          // permittivity
          NumericType epsr = matlib_()->get_parameter_value(device_.material(adjacent_semiconductor_segment_index), viennamini::mat::permittivity());
          viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
            epsr * viennamini::eps0::val());

        #ifdef VIENNAMINI_DEBUG
          std::cout << "segment " << current_segment_index << " is a contact (-semiconductor) segment " << std::endl;
          std::cout << "  contact value: " << device_.contact_potential(current_segment_index) << " workfunction: " << device_.workfunction(current_segment_index) << std::endl;
        #endif

          // potential dirichlet boundary
          viennafvm::set_dirichlet_boundary(potential, 
                                            segmesh.segmentation(current_segment_index), 
                                            device_.contact_potential(current_segment_index) + device_.workfunction(current_segment_index));
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
        numeric epsr = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
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
        NumericType epsr        = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
      
      #ifdef VIENNAMINI_DEBUG
        std::cout << "segment " << current_segment_index << " is a semiconductor segment " << std::endl;
        std::cout << "  epsr " << epsr << std::endl;
      #endif
        // potential
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
        
        // permittivity
        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
          epsr * viennamini::eps0::val());
      }
    }

    // -------------------------------------------------------------------------
    //
    // Specify partial differential equations
    //
    // -------------------------------------------------------------------------

    // here is all the fun: specify DD system
    EquationType laplace_eq = viennamath::make_equation( viennamath::div(permittivity_ * viennamath::grad(psi_)), /* = */ 0);

    // Specify the PDE system:
    viennafvm::linear_pde_system<> pde_system;
    pde_system.add_pde(laplace_eq, psi_); 
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


  FunctionSymbolType psi_;
  FunctionSymbolType n_;
  FunctionSymbolType p_;
  FunctionSymbolType permittivity_;
  FunctionSymbolType donator_doping_;
  FunctionSymbolType acceptor_doping_;
};

} // viennamini

#endif


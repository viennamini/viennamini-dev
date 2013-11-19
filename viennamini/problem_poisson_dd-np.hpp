#ifndef VIENNAMINI_PROBLEMPOISSONDDNP_HPP
#define VIENNAMINI_PROBLEMPOISSONDDNP_HPP

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

namespace viennamini {

struct problem_poisson_dd_np : public problem
{
public:
  problem_poisson_dd_np(viennamini::device& device, viennamini::config& config, viennamini::material_library& matlib) :
    problem         (device, config, matlib),
    psi_            (0),
    n_              (1),
    p_              (2),
    permittivity_   (3),
    donator_doping_ (4),
    acceptor_doping_(5)
  {
    
  }

  void run()
  {
//    if(device_.is_triangular2d())
//    {
//      generic_pde_solver_ = pde_solver_triangular_2d(device_.get_segmesh_triangular_2d().mesh);
//      this->run_impl(device_.get_segmesh_triangular_2d(), boost::get<pde_solver_triangular_2d>(generic_pde_solver_));
//    }
//    else 
//    if(device_.is_tetrahedral3d())
//    {
//      generic_pde_solver_ = pde_solver_tetrahedral_3d(device_.get_segmesh_tetrahedral_3d().mesh);
//      this->run_impl(device_.get_segmesh_tetrahedral_3d(), boost::get<pde_solver_tetrahedral_3d>(generic_pde_solver_));
//    }
//    else
//      std::cout << "detect_interfaces: segmented mesh type not supported" << std::endl;
  }
  
  
private:
  
  template<typename SegmentedMeshT, typename PDESolverT>
  void run_impl(SegmentedMeshT& segmesh, PDESolverT& pde_solver)
  {
//    typedef typename SegmentedMeshT::mesh_type                MeshType;
//    typedef typename SegmentedMeshT::segmentation_type        SegmentationType;
//    typedef viennafvm::pde_solver<MeshType>                   ProblemDescriptionType;
//    typedef typename ProblemDescriptionType::quantity_type    QuantityType;

////    ProblemDescriptionType problem_description(segmesh.mesh);
//    
//    std::size_t cell_num = viennagrid::cells(segmesh.mesh).size();
//    
//    QuantityType potential        (viennamini::id::potential(),        cell_num);
//    QuantityType electron_density (viennamini::id::electron_density(), cell_num);
//    QuantityType hole_density     (viennamini::id::hole_density(),     cell_num);
//    QuantityType permittivity     (viennamini::id::permittivity(),     cell_num);
//    QuantityType donator_doping   (viennamini::id::donator_doping(),   cell_num);
//    QuantityType acceptor_doping  (viennamini::id::acceptor_doping(),  cell_num);
//    
//    // -------------------------------------------------------------------------
//    //
//    // Assign segment roles: setup initial guesses and boundary conditions
//    //
//    // -------------------------------------------------------------------------
//    
//    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin(); 
//        sit != segmesh.segmentation.end(); ++sit)
//    {
//      std::size_t current_segment_index = sit->id();
//      
//      if(device_.is_contact(current_segment_index))
//      {
//        if(device_.is_contact_at_oxide(current_segment_index))
//        {
//          // potential dirichlet boundary
//          viennafvm::set_dirichlet_boundary(potential, 
//                                            segmesh.segmentation(current_segment_index), 
//                                            config_.contact_value(current_segment_index) + config_.workfunction(current_segment_index));
//        }
//        else
//        if(device_.is_contact_at_semiconductor(current_segment_index))
//        {
//          std::size_t adjacent_semiconductor_segment_index = device_.get_adjacent_semiconductor_segment_for_contact(current_segment_index);
//          NumericType ND_value    = device_.NA_max(adjacent_semiconductor_segment_index);
//          NumericType NA_value    = device_.ND_max(adjacent_semiconductor_segment_index);
//          NumericType builtin_pot = viennamini::built_in_potential(config_.temperature(), ND_value, NA_value);

//          // potential dirichlet boundary
//          viennafvm::set_dirichlet_boundary(potential, 
//                                            segmesh.segmentation(current_segment_index), 
//                                            config_.contact_value(current_segment_index) + config_.workfunction(current_segment_index) + builtin_pot);

//          // electrons dirichlet boundary
//          viennafvm::set_dirichlet_boundary(electron_density, segmesh.segmentation(current_segment_index), ND_value);

//          // holes dirichlet boundary
//          viennafvm::set_dirichlet_boundary(hole_density, segmesh.segmentation(current_segment_index), NA_value);
//        }
//        else
//        {
//          std::cout << "Error: Contact segment " << current_segment_index << 
//            " is neither sharing an interface with an oxide nor with a semiconductor" << std::endl;
//          exit(-1);
//        }
//      }
//      if(device_.is_oxide(current_segment_index))
//      {
//        // permittivity
//        numeric epsr = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
//        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
//          epsr * viennamini::eps0::val());
//      
//        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
//      }
//      if(device_.is_semiconductor(current_segment_index))
//      {
//        NumericType ND_value    = device_.NA_max(current_segment_index);
//        NumericType NA_value    = device_.ND_max(current_segment_index);
//        NumericType builtin_pot = viennamini::built_in_potential(config_.temperature(), ND_value, NA_value);
//        NumericType epsr        = matlib_()->get_parameter_value(device_.material(current_segment_index), viennamini::mat::permittivity());
//      
//        // potential
//        viennafvm::set_initial_value(potential, segmesh.segmentation(current_segment_index), builtin_pot);
//        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
//        
//        // permittivity
//        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index),
//          epsr * viennamini::eps0::val());

//        // electrons
//        viennafvm::set_initial_value(electron_density, segmesh.segmentation(current_segment_index), ND_value);
//        viennafvm::set_unknown(electron_density, segmesh.segmentation(current_segment_index));

//        // holes
//        viennafvm::set_initial_value(hole_density, segmesh.segmentation(current_segment_index), NA_value);
//        viennafvm::set_unknown(hole_density, segmesh.segmentation(current_segment_index));
//        
//        // donator doping
//        viennafvm::set_initial_value(donator_doping, segmesh.segmentation(current_segment_index), ND_value);

//        // acceptor doping
//        viennafvm::set_initial_value(acceptor_doping, segmesh.segmentation(current_segment_index), NA_value);
//      }
//    }
//    pde_solver.add_quantity(potential);
//    pde_solver.add_quantity(electron_density);
//    pde_solver.add_quantity(hole_density);
//    pde_solver.add_quantity(permittivity);
//    pde_solver.add_quantity(donator_doping);
//    pde_solver.add_quantity(acceptor_doping);

//    // -------------------------------------------------------------------------
//    //
//    // Specify partial differential equations
//    //
//    // -------------------------------------------------------------------------

//    // TODO: outsource to constants.hpp and physics.hpp
//    NumericType q  = 1.6e-19;
//    NumericType kB = 1.38e-23;
//    NumericType mu = 1;
//    NumericType T  = 300;
//    NumericType VT = kB * T / q;
//    NumericType D  = mu * VT;

//    // here is all the fun: specify DD system
//    EquationType poisson_eq = viennamath::make_equation( viennamath::div(permittivity_ * viennamath::grad(psi_)),                     /* = */ q * ((n_ - donator_doping_) - (p_ - acceptor_doping_)));
//    EquationType cont_eq_n  = viennamath::make_equation( viennamath::div(D * viennamath::grad(n_) - mu * viennamath::grad(psi_) * n_), /* = */ 0);
//    EquationType cont_eq_p  = viennamath::make_equation( viennamath::div(D * viennamath::grad(p_) + mu * viennamath::grad(psi_) * p_), /* = */ 0);

//    // Specify the PDE system:
//    viennafvm::linear_pde_system<> pde_system;
//    pde_system.add_pde(poisson_eq, psi_); pde_system.option(0).damping_term( (n_ + p_) * (-q / VT) );
//    pde_system.add_pde(cont_eq_n,  n_);   pde_system.option(1).geometric_update(true);
//    pde_system.add_pde(cont_eq_p,  p_);   pde_system.option(2).geometric_update(true);

//    pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running

//    // -------------------------------------------------------------------------
//    //
//    // Assemble and solve the problem
//    //
//    // -------------------------------------------------------------------------
//    viennafvm::linsolv::viennacl  linear_solver;
//    pde_solver(pde_system, linear_solver);
  }

  void write(std::string const& filename)
  {
  
  
//    viennafvm::io::write_solution_to_VTK_file(problem_description.quantities(), "device", segmesh.mesh, segmesh.segmentation);
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


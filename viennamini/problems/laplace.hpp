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
  VIENNAMINI_PROBLEM(problem_laplace)


  template<typename SegmentedMeshT, typename ProblemDescriptionSetT>
  void run_impl(SegmentedMeshT& segmesh,
                ProblemDescriptionSetT& problem_description_set,
                segment_values&         current_contact_potentials,
                std::size_t             step_id)
  {
    typedef typename SegmentedMeshT::segmentation_type        SegmentationType;
    typedef typename ProblemDescriptionSetT::value_type       ProblemDescriptionType;
    typedef typename ProblemDescriptionType::quantity_type    QuantityType;


    // -------------------------------------------------------------------------
    //
    // Extract ViennaFVM::Quantities
    //
    // -------------------------------------------------------------------------
    QuantityType & permittivity_initial      = problem_description_set[0].get_quantity(viennamini::id::permittivity());
     stream()  << "running laplace step: " << step_id << std::endl;
     stream() << " problem description set size: " << problem_description_set.size() << std::endl;
    ProblemDescriptionType& problem_description = problem_description_set[step_id];

    QuantityType & permittivity             = problem_description.add_quantity(permittivity_initial);
    if(viennamini::is_zero(permittivity.get_sum()))  throw required_quantity_is_zero_exception("Permittivity is zero!");

    QuantityType & potential                = problem_description.add_quantity(viennamini::id::potential());

    // -------------------------------------------------------------------------
    //
    // Assign segment roles: setup initial guesses and boundary conditions
    //
    // -------------------------------------------------------------------------

    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin();
        sit != segmesh.segmentation.end(); ++sit)
    {
      std::size_t current_segment_index = sit->id();

//    #ifdef VIENNAMINI_VERBOSE
      stream() << std::endl;
      stream() << "[Problem][Laplace] Processing segment " << current_segment_index << std::endl;
      stream() << "  Name:     \"" << device().get_name(current_segment_index) << "\"" << std::endl;
      stream() << "  Material: \"" << device().get_material(current_segment_index) << "\"" << std::endl;
//    #endif

      // each segment, even contacts, require a permittivity
      if(!device().has_permittivity(current_segment_index)) throw required_quantity_missing("Permittivity is not available on segment \""+device().get_name(current_segment_index)+"\"");

      if(device().is_contact(current_segment_index))
      {
        stream() << "  identified as a contact .." << std::endl;
        if(device().is_contact_at_semiconductor(current_segment_index))
        {
//        #ifdef VIENNAMINI_VERBOSE
          stream() << "  identified as a contact next to a semiconductor .." << std::endl;
//        #endif

//        #ifdef VIENNAMINI_VERBOSE
          stream() << "  pot:          " << current_contact_potentials[current_segment_index] << std::endl;
          stream() << "  workfunction: " << simulator().contact_workfunction(current_segment_index) << std::endl;
//        #endif

          // potential dirichlet boundary
          viennafvm::set_dirichlet_boundary(potential, segmesh.segmentation(current_segment_index),
            current_contact_potentials[current_segment_index] +
            simulator().contact_workfunction(current_segment_index)
          );
        }
        else
        if(device().is_contact_at_oxide(current_segment_index))
        {
//        #ifdef VIENNAMINI_VERBOSE
          stream() << "  identified as a contact next to an oxide .." << std::endl;
//        #endif

//        #ifdef VIENNAMINI_VERBOSE
          stream() << "  pot:          " << current_contact_potentials[current_segment_index] << std::endl;
          stream() << "  workfunction: " << simulator().contact_workfunction(current_segment_index) << std::endl;
//        #endif

          // potential dirichlet boundary
          viennafvm::set_dirichlet_boundary(potential, segmesh.segmentation(current_segment_index),
            current_contact_potentials[current_segment_index] +
            simulator().contact_workfunction(current_segment_index)
          );
        }
        else throw segment_undefined_contact_exception(current_segment_index);
      }
      else
      if(device().is_oxide(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "  identified as an oxide .." << std::endl;
      #endif
        viennafvm::set_unknown(potential, segmesh.segmentation(current_segment_index));
      }
      else
      if(device().is_semiconductor(current_segment_index))
      {
      #ifdef VIENNAMINI_VERBOSE
        stream() << "  identified as a semiconductor .." << std::endl;
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
    FunctionSymbolType eps  (permittivity.id());

    EquationType laplace_eq = viennamath::make_equation( viennamath::div(eps * viennamath::grad(psi)), /* = */ 0);

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
    linear_solver.break_tolerance() = config().linear_breaktol();
    linear_solver.max_iterations()  = config().linear_iterations();

    viennafvm::pde_solver pde_solver;

    if(config().write_initial_guess_files())
      this->write("initial_"+viennamini::convert<std::string>()(step_id), step_id);

  #ifdef VIENNAMINI_VERBOSE
    stream() << std::endl;
    stream() << "[Problem][Laplace] solving .. " << std::endl;
    stream() << std::endl;
  #endif

    pde_solver(problem_description, pde_system, linear_solver);
  }
};

} // viennamini

#endif


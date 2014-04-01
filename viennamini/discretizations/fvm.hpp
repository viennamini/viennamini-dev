#ifndef VIENNAMINI_DISCRETIZATIONS_FVM_HPP
#define VIENNAMINI_DISCRETIZATIONS_FVM_HPP

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

#include "viennamini/forwards.h"
#include "viennamini/discretization.hpp"

 //ViennaFVM includes:
#ifdef VIENNAMINI_VERBOSE
  #define VIENNAFVM_VERBOSE
#endif
#include "viennafvm/pde_solver.hpp"
#include "viennafvm/problem_description.hpp"
#include "viennafvm/forwards.h"
#include "viennafvm/boundary.hpp"
#include "viennafvm/io/vtk_writer.hpp"

namespace viennamini {

class fvm : public viennamini::discretization
{
public:

  VIENNAMINI_DISCRETIZATION(fvm)

  template<typename SegmentedMeshT>
  void run(SegmentedMeshT & segmesh)
  {
    typedef typename SegmentedMeshT::segmentation_type                           SegmentationType;
    typedef viennafvm::problem_description<typename SegmentedMeshT::mesh_type>   ProblemDescriptionType;
    typedef std::vector<ProblemDescriptionType>                                  ProblemDescriptionSetType;
    typedef typename ProblemDescriptionType::quantity_type                       QuantityType;

    ProblemDescriptionSetType  pbdesc_set;

    pbdesc_set.push_back(ProblemDescriptionType(segmesh.mesh));
    ProblemDescriptionType & current_pbdesc = pbdesc_set.back();

    initialize(segmesh, current_pbdesc);

    for(viennamini::pde_set::ids_type::iterator unknown_iter = config().model().get_pde_set().unknowns().begin();
        unknown_iter != config().model().get_pde_set().unknowns().end(); unknown_iter++)
    {
    #ifdef VIENNAMINI_VERBOSE
      stream() << "processing unknown: " << *unknown_iter << std::endl;
    #endif
      QuantityType & quan = current_pbdesc.add_quantity(*unknown_iter);
      config().model().get_pde_set().register_quantity(quan.get_name(), quan.id());

      for(typename SegmentationType::iterator sit = segmesh.segmentation.begin();
          sit != segmesh.segmentation.end(); ++sit)
      {
        std::size_t current_segment_index = sit->id();

      #ifdef VIENNAMINI_VERBOSE
        stream() << "  processing segment " << current_segment_index << ": \"" << device().get_name(current_segment_index) << "\"" << std::endl;
      #endif

        viennamini::role::segment_role_ids role = device().get_segment_role(current_segment_index);

      #ifdef VIENNAMINI_VERBOSE
        stream() << "    identified role: " << device().get_segment_role_string(current_segment_index) << std::endl;
      #endif

        // process only contact segments
        //
        if(role == viennamini::role::contact)
        {
          // if the PDE set has a contact model for this segment, apply it
          // i.e., overwrite the corresponding contact value on the device
          //
          if(config().model().get_pde_set().has_contact_model(*unknown_iter))
          {
            #ifdef VIENNAMINI_VERBOSE
              std::cout << "    applying contact model from PDE set .. " << std::endl;
            #endif
            config().model().get_pde_set().get_contact_model(*unknown_iter)->apply(device_handle(), current_segment_index);
          }

          // apply the contact value stored on the device as a Dirichlet contact
          // first evaluate whether the current unknown is to be solved on the neighbour segment,
          // otherwise we cannot expect a contact value to be present on the device
          //
          if(config().model().get_pde_set().is_role_supported(*unknown_iter, device().get_segment_role(device().get_adjacent_segment_for_contact(current_segment_index))))
          {
            // sanity check: make sure that a contact value is available ..
            if(device().has_contact(*unknown_iter, current_segment_index))
            {
              #ifdef VIENNAMINI_VERBOSE
                std::cout << "    assigning Dirichlet boundary value: " << device().get_contact(*unknown_iter, current_segment_index) << std::endl;
              #endif
              viennafvm::set_dirichlet_boundary(quan, segmesh.segmentation(current_segment_index), device().get_contact(*unknown_iter, current_segment_index));
            }
            else throw discretization_exception("Contact boundary condition for unknown \""+*unknown_iter+
              "\" is not available on segment "+viennamini::convert<std::string>(current_segment_index)+":\""+device().get_name(current_segment_index)+"\"");
          }
        }

        // Register 'unknowns' and set initial guesses if required if the current unknown
        // 'supports' the current segment role
        //
        if(config().model().get_pde_set().is_role_supported(*unknown_iter, role))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "    solving 'unknown' on this segment .." << std::endl;
        #endif
          viennafvm::set_unknown(quan, segmesh.segmentation(current_segment_index));

          // Oxides do not require an initial guess.
          // TODO: maybe move this 'logic' to the pde set
          //
          if(role == viennamini::role::oxide) continue;

          // if we deal with a nonlinear problem, we have to assign an initial guess
          if(!config().model().get_pde_set().is_linear())
          {
            // if there is a quantity distribution available, use it.
            // this way the user can conveniently 'inject' an initial guess
            //
            if(device().has_quantity(*unknown_iter, current_segment_index))
            {
            #ifdef VIENNAMINI_VERBOSE
              std::cout << "    using initial guess from device .. " << std::endl;
            #endif
              viennafvm::set_initial_value(quan, segmesh.segmentation(current_segment_index), device().get_quantity(*unknown_iter, current_segment_index));
            }
            //
            // otherwise ask the PDE set to provide an initial guess functor and forward it to ViennaFVM
            //
            else
            {
              viennafvm::set_initial_value(quan, segmesh.segmentation(current_segment_index), (config().model().get_pde_set().get_initial_guess(*unknown_iter, device_handle(), current_segment_index)));
            #ifdef VIENNAMINI_VERBOSE
              std::cout << "    using initial guess from PDE set .. " << std::endl;
            #endif
            }
          }
        }
      }
    }


    viennafvm::io::write_solution_to_VTK_file(
      current_pbdesc.quantities(),
      "initial",
      segmesh.mesh,
      segmesh.segmentation);

  #ifdef VIENNAMINI_VERBOSE
    std::cout << "preparing PDE system .." << std::endl;
  #endif
    viennafvm::linear_pde_system<> pde_system;
    pde_system.is_linear(config().model().get_pde_set().is_linear());
    typedef typename viennamini::pde_set::pdes_type PDEsType;
    PDEsType pdes = config().model().get_pde_set().get_pdes();
    int pde_index = 0;
    for(PDEsType::iterator pde_iter = pdes.begin(); pde_iter != pdes.end(); pde_iter++)
    {
      pde_system.add_pde(pde_iter->equation(), pde_iter->function_symbol());
      pde_system.option(pde_index).damping_term( pde_iter->damping_term() );
      pde_system.option(pde_index).geometric_update( pde_iter->geometric_update() );
      pde_index++;
    }

    // -------------------------------------------------------------------------
    //
    // Assemble and solve the problem
    //
    // -------------------------------------------------------------------------
  #ifdef VIENNAMINI_VERBOSE
    std::cout << "preparing solvers .." << std::endl;
  #endif
    viennafvm::linsolv::viennacl  linear_solver;
    linear_solver.break_tolerance() = config().linear_breaktol();
    linear_solver.max_iterations()  = config().linear_iterations();

    viennafvm::pde_solver pde_solver;
    pde_solver.set_nonlinear_iterations(config().nonlinear_iterations());
    pde_solver.set_nonlinear_breaktol(config().nonlinear_breaktol());

    // temporary fix to ensure proper handling of minority carriers
    // atm the damping must not be 1.0, so to be sure, we limit the damping to 0.9
    // see ViennaFVM commit:
    // https://github.com/viennafvm/viennafvm-dev/commit/3144e05af36be3beb02ee9be85d6d07c34031395
    if(config().damping() > 0.9)
    {
      config().damping() = 0.9;
    #ifdef VIENNAMINI_VERBOSE
      std::cout << "  non-linear solver damping too large, limiting damping to " << config().damping() << std::endl;
    #endif
    }
    else if(config().damping() <= 0.0)
    {
      config().damping() = 0.5;
    #ifdef VIENNAMINI_VERBOSE
      std::cout << "  non-linear solver damping too small, limiting damping to " << config().damping() << std::endl;
    #endif
    }
    pde_solver.set_damping(config().damping());

  #ifdef VIENNAMINI_VERBOSE
    std::cout << "assembling and solving the system .." << std::endl;
  #endif
    bool converged = pde_solver(current_pbdesc, pde_system, linear_solver);

    if(converged) stream() << "  simulation did converge!" << std::endl;
    else          stream() << "  simulation did NOT converge!" << std::endl;

    viennafvm::io::write_solution_to_VTK_file(
      current_pbdesc.quantities(),
      "output",
      segmesh.mesh,
      segmesh.segmentation);

  }

private:

  template<typename SegmentedMeshT, typename ProblemDescriptionT>
  void initialize(SegmentedMeshT & segmesh, ProblemDescriptionT & pdesc)
  {
    typedef typename SegmentedMeshT::segmentation_type          SegmentationType;
    typedef typename ProblemDescriptionT::quantity_type         QuantityType;

    for(viennamini::pde_set::ids_type::iterator dep_iter = config().model().get_pde_set().dependencies().begin();
        dep_iter != config().model().get_pde_set().dependencies().end(); dep_iter++)
    {
    #ifdef VIENNAMINI_VERBOSE
      stream() << "processing dependency: " << *dep_iter << std::endl;
    #endif
      QuantityType & quan = pdesc.add_quantity(*dep_iter);
      config().model().get_pde_set().register_quantity(quan.get_name(), quan.id());

      for(typename SegmentationType::iterator sit = segmesh.segmentation.begin();
          sit != segmesh.segmentation.end(); ++sit)
      {
        std::size_t current_segment_index = sit->id();

      #ifdef VIENNAMINI_VERBOSE
        stream() << "  processing segment " << current_segment_index << ": \"" << device().get_name(current_segment_index) << "\"" << std::endl;
      #endif

        viennamini::role::segment_role_ids role = device().get_segment_role(current_segment_index);

      #ifdef VIENNAMINI_VERBOSE
        stream() << "    identified role: " << device().get_segment_role_string(current_segment_index) << std::endl;
      #endif

        if(config().model().get_pde_set().is_role_supported(*dep_iter, role))
        {
        #ifdef VIENNAMINI_VERBOSE
          stream() << "    role supported, distributing quantities .. " << std::endl;
        #endif
          if(device().has_quantity(*dep_iter, current_segment_index))
          {
            viennafvm::set_initial_value(quan, segmesh.segmentation(current_segment_index), device().get_quantity(*dep_iter, current_segment_index));
          }
          else throw discretization_exception("Quantity \""+*dep_iter+"\" is not available on segment "+viennamini::convert<std::string>(current_segment_index)+":\""+device().get_name(current_segment_index)+"\"");
        }
      }
    }
  }

  template<typename SegmentedMeshT, typename ProblemDescriptionSetT>
  void transfer_dependencies(SegmentedMeshT & segmesh, ProblemDescriptionSetT & pdesc_set)
  {
//    pdesc_set->back().add_quantity( (pdesc_set.end()-2)->get_quantity(viennamini::id::permittivity()) );
  }

};

} // viennamini


#endif


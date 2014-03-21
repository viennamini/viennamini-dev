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

// ViennaFVM includes:
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
  fvm(viennamini::device_handle        device, 
      viennamini::configuration_handle config, 
      viennamini::stepper_handle       stepper, 
      std::ostream                   & stream) :
    viennamini::discretization(device, config, stepper, stream) {}
  ~fvm() {}

  virtual void run() 
  {
    if(device().is_line1d())
      run_impl(device().get_segmesh_line_1d());
    else
    if(device().is_triangular2d())
      run_impl(device().get_segmesh_triangular_2d());
    else
    if(device().is_tetrahedral3d())
      run_impl(device().get_segmesh_tetrahedral_3d());
    else throw device_not_supported_exception("at: problem_laplace::run()");
  }

  template<typename SegmentedMeshT>
  void run_impl(SegmentedMeshT & segmesh)
  {
    typedef viennafvm::problem_description<typename SegmentedMeshT::mesh_type>   ProblemDescriptionType;
    typedef std::vector<ProblemDescriptionType>                                  ProblemDescriptionSetType;
    typedef typename ProblemDescriptionType::quantity_type                       QuantityType;

    ProblemDescriptionSetType  pbdesc_set;

    pbdesc_set.push_back(ProblemDescriptionType(segmesh.mesh));
    ProblemDescriptionType & current_pbdesc = pbdesc_set.back();

    initialize(segmesh, current_pbdesc);

//      QuantityType & permittivity = current_pbdesc.get_quantity(viennamini::id::permittivity());
//      QuantityType & potential    = current_pbdesc.add_quantity(viennamini::id::potential());



  }

private:

  template<typename SegmentedMeshT, typename ProblemDescriptionT>
  void initialize(SegmentedMeshT & segmesh, ProblemDescriptionT & pdesc)
  { 
    typedef typename SegmentedMeshT::segmentation_type          SegmentationType;
    typedef typename ProblemDescriptionT::quantity_type         QuantityType;

    for(viennamini::pde_set::ids_type::iterator dep_iter = config().model().pde_set().dependencies().begin();
        dep_iter != config().model().pde_set().dependencies().end(); dep_iter++)
    {
      std::cout << *dep_iter << std::endl;
      // QuantityType & quan = pdesc.add_quantity(*dep_iter);

      for(typename SegmentationType::iterator sit = segmesh.segmentation.begin();
          sit != segmesh.segmentation.end(); ++sit)
      {
//      std::size_t current_segment_index = sit->id();
//      if(device().has_quantity(*dep_iter, current_segment_index))
//      {
//        
//        viennafvm::set_initial_value(quan, segmesh.segmentation(current_segment_index), device().get_quantity(*dep_iter, current_segment_index));
//      }
      }
    }
    


//    QuantityType & permittivity = pdesc.add_quantity(viennamini::id::permittivity());

//    for(typename SegmentationType::iterator sit = segmesh.segmentation.begin();
//        sit != segmesh.segmentation.end(); ++sit)
//    {
//      std::size_t current_segment_index = sit->id();
//      if(device().has_permittivity(current_segment_index))
//      {
//        viennafvm::set_initial_value(permittivity, segmesh.segmentation(current_segment_index), device().get_permittivity(current_segment_index));
//      }
//    }
  }

  template<typename SegmentedMeshT, typename ProblemDescriptionSetT>
  void transfer_dependencies(SegmentedMeshT & segmesh, ProblemDescriptionSetT & pdesc_set)
  {
//    pdesc_set->back().add_quantity( (pdesc_set.end()-2)->get_quantity(viennamini::id::permittivity()) );
  }

};

} // viennamini


#endif


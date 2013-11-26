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


// ViennaMini includes
#include "viennamini/simulator.hpp"
#include "viennamini/device_collection.hpp"


namespace viennamini {

struct problem_poisson : public problem
{
  VIENNAMINI_PROBLEM

  
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
      std::cout << "[Problem][Poisson] Processing segment " << current_segment_index << std::endl;
      std::cout << "  Name:     \"" << device().get_name(current_segment_index) << "\"" << std::endl;
      std::cout << "  Material: \"" << device().get_material(current_segment_index) << "\"" << std::endl;
    #endif

      if(device().is_contact(current_segment_index))
      {
        if(device().is_contact_at_semiconductor(current_segment_index))
        {
        #ifdef VIENNAMINI_VERBOSE
          std::cout << "  identified as a contact next to a semiconductor .." << std::endl;
        #endif
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

                                                                       /* LHS */                               /* RHS */
    EquationType laplace_eq = viennamath::make_equation( viennamath::div(epsr * viennamath::grad(psi)), /* = */ epsr * 5.0);

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

    if(config().write_initial_guesses())
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

void setup_simulation(viennamini::simulator&  mysim)
{
  mysim.device().read(viennamini::device_collection_path()+"/nin2d/nin2d.mesh", viennamini::triangular_2d());
  mysim.device().read_material_library("../../examples/materials.xml");
  mysim.device().scale(1.0E-9);

  const int left_contact     = 1;
  const int left             = 2;
  const int intrinsic        = 3;
  const int right            = 4;
  const int right_contact    = 5;

  mysim.device().make_contact         (left_contact);
  mysim.device().set_name             (left_contact, "left_contact");
  mysim.device().set_material         (left_contact, "Cu");
  mysim.device().set_contact_potential(left_contact, 0.0);

  mysim.device().make_semiconductor   (left);
  mysim.device().set_name             (left, "left");
  mysim.device().set_material         (left, "Si");

  mysim.device().make_semiconductor   (intrinsic);
  mysim.device().set_name             (intrinsic, "left");
  mysim.device().set_material         (intrinsic, "Si");

  mysim.device().make_semiconductor   (right);
  mysim.device().set_name             (right, "right");
  mysim.device().set_material         (right, "Si");

  mysim.device().make_contact         (right_contact);
  mysim.device().set_name             (right_contact, "right_contact");
  mysim.device().set_material         (right_contact, "Cu");
  mysim.device().set_contact_potential(right_contact, 0.2);

  mysim.config().temperature()                        = 300;
  mysim.config().linear_breaktol()                    = 1.0E-14;
  mysim.config().linear_iterations()                  = 1000;
  mysim.config().write_initial_guesses()              = true;

}

int main()
{
  //
  // prepare the simulator object and setup some example device 
  //
  viennamini::simulator  mysim;
  setup_simulation(mysim);

  //
  // instantiate the new 'Poisson' problem and pass it on to the simulator 
  //
  mysim.set_problem(new viennamini::problem_poisson());

  //
  // run the simulation
  //
  mysim.run();
  
  mysim.write("new_problem_result");

  std::cout << "*************************************************" << std::endl;
  std::cout << "* New Problem simulation finished successfully! *" << std::endl;
  std::cout << "*************************************************" << std::endl;
  return EXIT_SUCCESS;
}

/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
           ViennaFVM - The Vienna Finite Volume Method Library
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               (add your name here)

   license:    To be discussed, see file LICENSE in the ViennaFVM base directory
======================================================================= */

//#define VIENNAFVM_DEBUG

// Define NDEBUG to get any reasonable performance with ublas:
#define NDEBUG

// include necessary system headers
#include <iostream>

// ViennaMini main include:
#include "viennamini/simulator.hpp"


template <typename DomainType>
void init_quantities(DomainType const & my_domain, double n_plus, double p_plus)
{
  //
  // Init permittivity
  //
  viennafvm::set_quantity_region(permittivity_key(), my_domain, true);               // permittivity is (for simplicity) defined everywhere
  viennafvm::set_quantity_value(permittivity_key(), my_domain, 11.7 * 8.854e-12);                // permittivity of silicon
  viennafvm::set_quantity_value(permittivity_key(), my_domain.segments()[2], 15.6 * 8.854e-12);  // permittivty of HfO2

  //
  // Initialize doping
  //

  // donator doping
  viennafvm::set_quantity_region(donator_doping_key(), my_domain.segments()[4], true);    // source
  viennafvm::set_quantity_region(donator_doping_key(), my_domain.segments()[5], true);    // drain
  viennafvm::set_quantity_region(donator_doping_key(), my_domain.segments()[6], true);    // body
  viennafvm::set_quantity_region(donator_doping_key(), my_domain.segments()[7], true);    // body contact (floating body)

  viennafvm::set_quantity_value(donator_doping_key(), my_domain.segments()[4],  n_plus);      // source
  viennafvm::set_quantity_value(donator_doping_key(), my_domain.segments()[5],  n_plus);      // drain
  viennafvm::set_quantity_value(donator_doping_key(), my_domain.segments()[6],  1e32/p_plus); // body
  viennafvm::set_quantity_value(donator_doping_key(), my_domain.segments()[7],  1e32/p_plus); // body contact (floating body)

  // acceptor doping
  viennafvm::set_quantity_region(acceptor_doping_key(), my_domain.segments()[4], true);   // source
  viennafvm::set_quantity_region(acceptor_doping_key(), my_domain.segments()[5], true);   // drain
  viennafvm::set_quantity_region(acceptor_doping_key(), my_domain.segments()[6], true);   // body
  viennafvm::set_quantity_region(acceptor_doping_key(), my_domain.segments()[7], true);   // body contact (floating body)

  viennafvm::set_quantity_value(acceptor_doping_key(), my_domain.segments()[4],  1e32/n_plus); // source
  viennafvm::set_quantity_value(acceptor_doping_key(), my_domain.segments()[5],  1e32/n_plus); // drain
  viennafvm::set_quantity_value(acceptor_doping_key(), my_domain.segments()[6],  p_plus);      // body
  viennafvm::set_quantity_value(acceptor_doping_key(), my_domain.segments()[7],  p_plus);      // body contact (floating body)

  // built-in potential:
  viennafvm::set_quantity_region(builtin_potential_key(), my_domain, true);   // defined everywhere

  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[0], built_in_potential(300, n_plus, 1e32/n_plus)); // gate
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[1], built_in_potential(300, n_plus, 1e32/n_plus)); // source contact
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[2], built_in_potential(300, n_plus, 1e32/n_plus)); // oxide (for simplicity set to same as gate)
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[3], built_in_potential(300, n_plus, 1e32/n_plus)); // drain contact
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[4], built_in_potential(300, n_plus, 1e32/n_plus)); // source
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[5], built_in_potential(300, n_plus, 1e32/n_plus)); // drain
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[6], built_in_potential(300, 1e32/p_plus, p_plus)); // body
  viennafvm::set_quantity_value(builtin_potential_key(), my_domain.segments()[7], built_in_potential(300, 1e32/p_plus, p_plus)); // body contact (floating body)
}

/** @brief Scales the entire simulation domain (device) by the provided factor. This is accomplished by multiplying all point coordinates with this factor. */
template <typename DomainType>
void scale_domain(DomainType & domain, double factor)
{
  typedef typename viennagrid::result_of::ncell_range<DomainType, 0 > ::type VertexContainer;
  typedef typename viennagrid::result_of::iterator<VertexContainer>::type VertexIterator;

  VertexContainer vertices = viennagrid::ncells < 0 > (domain);
  for ( VertexIterator vit = vertices.begin();
        vit != vertices.end();
        ++vit )
  {
    vit->point() *= factor; // scale
  }
}

int main()
{
  typedef double   numeric_type;

  typedef viennagrid::config::triangular_2d                           ConfigType;
  typedef viennagrid::result_of::domain<ConfigType>::type             DomainType;
  typedef typename ConfigType::cell_tag                     CellTag;

  typedef viennagrid::result_of::ncell<ConfigType, CellTag::dim>::type        CellType;

  typedef viennamath::function_symbol   FunctionSymbol;
  typedef viennamath::equation          Equation;

  //
  // Create a domain from file
  //
  DomainType my_domain;

  try
  {
    viennagrid::io::netgen_reader my_reader;
    my_reader(my_domain, "../examples/data/mosfet.mesh");
  }
  catch (...)
  {
    std::cerr << "File-Reader failed. Aborting program..." << std::endl;
    return EXIT_FAILURE;
  }

  scale_domain(my_domain, 1e-9); // scale to nanometer

  //
  // Set initial values
  //
  double n_plus = 1e24;
  double p_plus = 1e20;

  init_quantities(my_domain, n_plus, p_plus);

  //
  // Setting boundary information on domain (see mosfet.in2d for segment indices)
  //
  FunctionSymbol psi(0);   // potential, using id=0
  FunctionSymbol n(1);     // electron concentration, using id=1
  FunctionSymbol p(2);     // hole concentration, using id=2

  // potential:
  double built_in_pot = built_in_potential(300, n_plus, 1e32/n_plus); // should match specification in init_quantities()!
  viennafvm::set_dirichlet_boundary(my_domain.segments()[0], 0.2 + built_in_pot, psi); // Gate contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[1], 0.0 + built_in_pot, psi); // Source contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[3], 0.2 + built_in_pot, psi); // Drain contact
  // using floating body, hence commented:
  viennafvm::set_dirichlet_boundary(my_domain.segments()[7], 0.0 + built_in_potential(300, 1e32/p_plus, p_plus), psi); // Body contact

  // electron density
  viennafvm::set_dirichlet_boundary(my_domain.segments()[1], n_plus, n); // Source contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[3], n_plus, n); // Drain contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[7], 1e32/p_plus, n); // Body contact

  // hole density
  viennafvm::set_dirichlet_boundary(my_domain.segments()[1], 1e32/n_plus, p); // Source contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[3], 1e32/n_plus, p); // Drain contact
  viennafvm::set_dirichlet_boundary(my_domain.segments()[7], p_plus, p); // Body contact


  //
  // Set quantity mask: By default, a quantity is defined on the entire domain.
  //                    Boundary equations are already specified above.
  //                    All that is left is to specify regions where a quantity 'does not make sense'
  //                    Here, we need to disable {n,p} in the gate oxide and the gate
  //

  viennafvm::disable_quantity(my_domain.segments()[0], n); // Gate contact
  viennafvm::disable_quantity(my_domain.segments()[2], n); // Gate oxide

  viennafvm::disable_quantity(my_domain.segments()[0], p); // Gate contact
  viennafvm::disable_quantity(my_domain.segments()[2], p); // Gate oxide

  //
  // Initial conditions (required for nonlinear problems)
  //
  viennafvm::set_initial_guess(my_domain, psi, builtin_potential_key());
  //viennafvm::smooth_initial_guess(my_domain, psi, viennafvm::arithmetic_mean_smoother());
  //viennafvm::smooth_initial_guess(my_domain, psi, viennafvm::arithmetic_mean_smoother());

  viennafvm::set_initial_guess(my_domain, n, donator_doping_key());
  //viennafvm::smooth_initial_guess(my_domain, n, viennafvm::geometric_mean_smoother());
  //viennafvm::smooth_initial_guess(my_domain, n, viennafvm::geometric_mean_smoother());

  viennafvm::set_initial_guess(my_domain, p, acceptor_doping_key());
  //viennafvm::smooth_initial_guess(my_domain, p, viennafvm::geometric_mean_smoother());
  //viennafvm::smooth_initial_guess(my_domain, p, viennafvm::geometric_mean_smoother());




  //
  // Writing all solution variables back to domain
  //
  std::vector<long> result_ids(3); //TODO: Better way to make potential, electron_density and hole_density accessible
  result_ids[0] = 0;
  result_ids[1] = 1;
  result_ids[2] = 2;

  viennafvm::io::write_solution_to_VTK_file(pde_solver.result(), "mosfet", my_domain, result_ids);

  std::cout << "********************************************" << std::endl;
  std::cout << "* MOSFET simulation finished successfully! *" << std::endl;
  std::cout << "********************************************" << std::endl;
  return EXIT_SUCCESS;
}


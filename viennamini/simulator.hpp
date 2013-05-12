#ifndef VIENNAMINI_SIMULATOR_HPP
#define VIENNAMINI_SIMULATOR_HPP

/* Come up with a simulator object similar to ViennaSHE.
   Configuration should happen in a similar manner, allowing for the selection of predefined models (DD, Hydro, ev. ET)
*/


// ViennaFVM includes:
#include "viennafvm/forwards.h"
#include "viennafvm/linear_assembler.hpp"
#include "viennafvm/io/vtk_writer.hpp"
#include "viennafvm/boundary.hpp"
#include "viennafvm/pde_solver.hpp"
#include "viennafvm/initial_guess.hpp"

// ViennaGrid includes:
#include "viennagrid/domain.hpp"
#include <viennagrid/config/simplex.hpp>
#include "viennagrid/io/netgen_reader.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/voronoi.hpp"

// ViennaData includes:
#include "viennadata/api.hpp"

// ViennaMath includes:
#include "viennamath/expression.hpp"

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_sparse.hpp>

#include "viennamini/fwd.h"
#include "viennamini/physics.hpp"


namespace viennamini
{
  class simulator
  {
   public:
    typedef viennamath::function_symbol   FunctionSymbol;
    typedef viennamath::equation          Equation;


    void operator()() const
    {

      //
      // Specify PDEs:
      //

      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  permittivity; permittivity.wrap_constant( permittivity_key() );
      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  donator_doping; donator_doping.wrap_constant( donator_doping_key() );
      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  acceptor_doping; acceptor_doping.wrap_constant( acceptor_doping_key() );

      double q  = 1.6e-19;
      double kB = 1.38e-23; // Boltzmann constant
      double mu = 1;        // mobility (constant is fine for the moment)
      double T  = 300;
      double VT = kB * T / q;
      double D  = mu * VT;  //diffusion constant

      // here is all the fun: specify DD system
      FunctionSymbol psi = quantity_potential();         // potential, using id=0
      FunctionSymbol n   = quantity_electron_density();  // electron concentration, using id=1
      FunctionSymbol p   = quantity_hole_density();      // hole concentration, using id=2

      Equation poisson_eq = viennamath::make_equation( viennamath::div(permittivity * viennamath::grad(psi)),                     /* = */ q * ((n - donator_doping) - (p - acceptor_doping)));
      Equation cont_eq_n  = viennamath::make_equation( viennamath::div(D * viennamath::grad(n) - mu * viennamath::grad(psi) * n), /* = */ 0);
      Equation cont_eq_p  = viennamath::make_equation( viennamath::div(D * viennamath::grad(p) + mu * viennamath::grad(psi) * p), /* = */ 0);

      // Specify the PDE system:
      viennafvm::linear_pde_system<> pde_system;
      pde_system.add_pde(poisson_eq, psi); // equation and associated quantity
      pde_system.add_pde(cont_eq_n, n);    // equation and associated quantity
      pde_system.add_pde(cont_eq_p, p);    // equation and associated quantity

      pde_system.option(0).damping_term( (n + p) * (-q / VT) );
      pde_system.option(1).geometric_update(true);
      pde_system.option(2).geometric_update(true);

      pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running


      //
      // Create PDE solver instance and run the solver:
      //

      pde_solver(pde_system, my_domain);   // weird math happening in here ;-)

    }

    FunctionSymbol quantity_potential()        const { return FunctionSymbol(0); }
    FunctionSymbol quantity_electron_density() const { return FunctionSymbol(1); }
    FunctionSymbol quantity_hole_density()     const { return FunctionSymbol(2); }

    

   private:
    viennafvm::pde_solver<> pde_solver;
  }
}

#endif 

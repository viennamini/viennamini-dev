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

// ViennaMaterials includes:
#include "viennamaterials/library.hpp"
#include "viennamaterials/kernels/pugixml.hpp"

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_sparse.hpp>

#include "viennamini/fwd.h"
#include "viennamini/physics.hpp"
#include "viennamini/constants.hpp"
#include "viennamini/config.hpp"
#include "viennamini/device.hpp"

namespace viennamini
{
  template<typename MatlibT>
  class simulator
  {
   public:
    typedef viennamath::function_symbol         function_symbol_type;
    typedef viennamath::equation                equation_type;
    typedef double                              numeric_type;
    typedef boost::numeric::ublas::vector<double>   vector_type;

    simulator(MatlibT& matlib) : matlib(matlib) {}

    template <typename DomainT>
    void operator()(viennamini::Device<DomainT, MatlibT> & device,
                    viennamini::Config                   & config )
    {
      // finalize the device setup
      //
      this->prepare(device, config);

      typedef viennamini::Device<DomainT, MatlibT> Device;




    }

private:
    template <typename DomainT>
    void prepare(viennamini::Device<DomainT, MatlibT> & device,
                 viennamini::Config                   & config )
    {
      typedef viennamini::Device<DomainT, MatlibT> Device;

      typedef typename Device::Indices Indices;

      Indices& contact_segments = device.get_contact_segments();
      for(typename Indices::iterator iter = contact_segments.begin();
          iter != contact_segments.end(); iter++)
      {

        viennafvm::set_dirichlet_boundary(device.get_domain().segments()[*iter],
            config.get_contact_values(*iter)[0], quantity_potential());

      //            viennafvm::set_dirichlet_boundary(domain.segments()[si],
      //                                              n_plus, my_simulator.quantity_electron_density());

      //            viennafvm::set_dirichlet_boundary(domain.segments()[si],
      //                                              p_plus, my_simulator.quantity_hole_density());
      }



      // now, finish preparing the oxides
      Indices& oxide_segments = device.get_oxide_segments();
      for(typename Indices::iterator iter = oxide_segments.begin();
          iter != oxide_segments.end(); iter++)
      {
        viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
        viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
      }

      // .. finalize the semiconductor segments too!
      //
      Indices& semiconductor_segments = device.get_semiconductor_segments();
      for(typename Indices::iterator iter = semiconductor_segments.begin();
          iter != semiconductor_segments.end(); iter++)
      {
        numeric_type build_int_potential_value = viennamini::built_in_potential(
                config.acc_temperature(), device.get_donator(*iter), device.get_acceptor(*iter));

        viennafvm::set_quantity_region(builtin_key, device.get_domain().segments()[*iter], true);
        viennafvm::set_quantity_value(builtin_key, device.get_domain().segments()[*iter],
                                      build_int_potential_value);
      }
    }


//    template <typename DomainType>
//    void operator()(DomainType const & my_domain)
//    {
//      typedef typename DomainType::config_type    ConfigType;
//      typedef typename ConfigType::cell_tag       CellTag;

//      typedef typename viennagrid::result_of::ncell<ConfigType, CellTag::dim>::type   CellType;

//      //
//      // Specify PDEs:
//      //

//      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  permittivity; permittivity.wrap_constant( permittivity_key() );
//      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  donator_doping; donator_doping.wrap_constant( donator_doping_key() );
//      viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  acceptor_doping; acceptor_doping.wrap_constant( acceptor_doping_key() );

//      double q  = 1.6e-19;
//      double kB = 1.38e-23; // Boltzmann constant
//      double mu = 1;        // mobility (constant is fine for the moment)
//      double T  = 300;
//      double VT = kB * T / q;
//      double D  = mu * VT;  //diffusion constant

//      // here is all the fun: specify DD system
//      function_symbol_type psi = quantity_potential();         // potential, using id=0
//      function_symbol_type n   = quantity_electron_density();  // electron concentration, using id=1
//      function_symbol_type p   = quantity_hole_density();      // hole concentration, using id=2

//      // Set up the Poisson equation and the two continuity equations
//      equation_type poisson_eq = viennamath::make_equation( viennamath::div(permittivity * viennamath::grad(psi)),                     /* = */ q * ((n - donator_doping) - (p - acceptor_doping)));
//      equation_type cont_eq_n  = viennamath::make_equation( viennamath::div(D * viennamath::grad(n) - mu * viennamath::grad(psi) * n), /* = */ 0);
//      equation_type cont_eq_p  = viennamath::make_equation( viennamath::div(D * viennamath::grad(p) + mu * viennamath::grad(psi) * p), /* = */ 0);

//      // Specify the PDE system:
//      viennafvm::linear_pde_system<> pde_system;
//      pde_system.add_pde(poisson_eq, psi); // equation and associated quantity
//      pde_system.add_pde(cont_eq_n, n);    // equation and associated quantity
//      pde_system.add_pde(cont_eq_p, p);    // equation and associated quantity

//      pde_system.option(0).damping_term( (n + p) * (-q / VT) );
//      pde_system.option(1).geometric_update(true);
//      pde_system.option(2).geometric_update(true);

//      pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running


//      //
//      // Create PDE solver instance and run the solver:
//      //

//      viennafvm::pde_solver<>  dd_solver;
//      dd_solver(pde_system, my_domain);   // weird math happening in here ;-)

//      // Get result vector:
//      result_ = dd_solver.result();
//    }

    function_symbol_type quantity_potential()        const { return function_symbol_type(0); }
    function_symbol_type quantity_electron_density() const { return function_symbol_type(1); }
    function_symbol_type quantity_hole_density()     const { return function_symbol_type(2); }

    vector_type const & result() const { return result_; }

   private:
    viennafvm::pde_solver<> pde_solver;
    vector_type             result_;
    MatlibT&                matlib;

    viennamini::builtin_potential_key  builtin_key;

  };
}

#endif 

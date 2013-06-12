#ifndef VIENNAMINI_SIMULATOR_HPP
#define VIENNAMINI_SIMULATOR_HPP

/* Come up with a simulator object similar to ViennaSHE.
   Configuration should happen in a similar manner, allowing for the selection of predefined models (DD, Hydro, ev. ET)
*/

#ifndef NDEBUG
  #define NDEBUG
#endif

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
#include "viennagrid/algorithm/interface.hpp"

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
#include "viennamini/initial_guess_accessor.hpp"
#include "viennamini/result_accessor.hpp"

namespace viennamini
{
    static int NOTFOUND = -1;


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
            // detect contact-semiconductor and contact-oxide interfaces
            //
            this->detect_interfaces(device);

            // finalize the device setup
            //
            this->prepare(device, config);

//            // write doping and initial guesses (including boundary conditions) to
//            // vtk files for analysis
//            //
//            this->write_device_doping(device);
//            this->write_device_initial_guesses(device);

            // run the simulation
            //
            this->run(device, config);
        }

        template <typename DomainT>
        void write_device_doping(viennamini::Device<DomainT, MatlibT> & device)
        {

            viennagrid::io::vtk_writer<DomainT> my_vtk_writer;
            viennagrid::io::add_scalar_data_on_cells<viennamini::donator_doping_key,   double, DomainT>(my_vtk_writer, viennamini::donator_doping_key(),   "donators");
            viennagrid::io::add_scalar_data_on_cells<viennamini::acceptor_doping_key,  double, DomainT>(my_vtk_writer, viennamini::acceptor_doping_key(),  "acceptors");
            my_vtk_writer(device.get_domain(), "viennamini_doping");
        }

        template <typename DomainT>
        void write_device_initial_guesses(viennamini::Device<DomainT, MatlibT> & device)
        {
            typedef viennafvm::boundary_key             BoundaryKey;
            typedef viennafvm::current_iterate_key      IterateKey;

            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_pot(quantity_potential().id());
            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_n(quantity_electron_density().id());
            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_p(quantity_hole_density().id());

            typedef typename DomainT::config_type                                           ConfigType;
            typedef typename ConfigType::cell_tag                                           CellTag;
            typedef typename viennagrid::result_of::const_ncell_range<DomainT, CellTag::dim>::type      CellContainer;
            typedef typename viennagrid::result_of::iterator<CellContainer>::type                       CellIterator;
            CellContainer const& cells = viennagrid::ncells<CellTag::dim>(device.get_domain());
            for(CellIterator cit = cells.begin(); cit != cells.end(); cit++)
            {
                viennadata::access<std::string, double>("initial_pot")(*cit) = init_guess_pot(*cit);
                viennadata::access<std::string, double>("initial_n")(*cit) = init_guess_n(*cit);
                viennadata::access<std::string, double>("initial_p")(*cit) = init_guess_p(*cit);
            }

            viennagrid::io::vtk_writer<DomainT> initial_writer;
            viennagrid::io::add_scalar_data_on_cells<std::string,double, DomainT>(initial_writer, "initial_pot", "initial_pot");
            viennagrid::io::add_scalar_data_on_cells<std::string,double, DomainT>(initial_writer, "initial_n", "initial_n");
            viennagrid::io::add_scalar_data_on_cells<std::string,double, DomainT>(initial_writer, "initial_p", "initial_p");
            initial_writer(device.get_domain(), "viennamini_initial_guesses");
        }

    private:

        template <typename DomainT>
        void detect_interfaces(viennamini::Device<DomainT, MatlibT> & device)
        {
            typedef viennamini::Device<DomainT, MatlibT>    Device;
            typedef typename Device::Indices                Indices;

            typedef typename DomainT::config_type                                                   ConfigType;
            typedef typename ConfigType::cell_tag                                                   CellTag;
            typedef typename viennagrid::result_of::segment<ConfigType>::type                       SegmentType;
            typedef typename viennagrid::result_of::ncell_range<SegmentType, CellTag::dim-1>::type  FacetRange;
            typedef typename viennagrid::result_of::iterator<FacetRange>::type                      FacetIterator;

            Indices& contact_segments       = device.get_contact_segments();
            Indices& oxide_segments         = device.get_oxide_segments();
            Indices& semiconductor_segments = device.get_semiconductor_segments();

            for(typename Indices::iterator cs_it = contact_segments.begin();
              cs_it != contact_segments.end(); cs_it++)
            {
                SegmentType& current_contact_segment = device.get_domain().segments()[*cs_it];

                int adjacent_semiconduct_segment_id = find_adjacent_segment(device, current_contact_segment, semiconductor_segments);
                if(adjacent_semiconduct_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Semiconductor segment #" << adjacent_semiconduct_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactSemiconductorInterfaces[*cs_it] = adjacent_semiconduct_segment_id;
                }

                int adjacent_oxide_segment_id = find_adjacent_segment(device, current_contact_segment, oxide_segments);
                if(adjacent_oxide_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Oxide segment #" << adjacent_oxide_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactOxideInterfaces[*cs_it] = adjacent_oxide_segment_id;
                }
            }
        }

        template <typename DomainT, typename SegmentType, typename IndicesT>
        int find_adjacent_segment(viennamini::Device<DomainT, MatlibT> & device,
                                  SegmentType                          & current_contact_segment,
                                  IndicesT                             & segments_under_test)
        {
            typedef typename DomainT::config_type                                                   ConfigType;
            typedef typename ConfigType::cell_tag                                                   CellTag;
            typedef typename viennagrid::result_of::ncell_range<SegmentType, CellTag::dim-1>::type  FacetRange;
            typedef typename viennagrid::result_of::iterator<FacetRange>::type                      FacetIterator;

            FacetRange   facets                  = viennagrid::ncells<CellTag::dim-1>(current_contact_segment);

            for(typename IndicesT::iterator sit = segments_under_test.begin();
                sit != segments_under_test.end(); sit++)
            {
                SegmentType& current_segment = device.get_domain().segments()[*sit];

                for (FacetIterator fit = facets.begin(); fit != facets.end(); ++fit)
                {
                    if (viennagrid::is_interface(*fit, current_contact_segment, current_segment))
                    {
                        return *sit;
                    }
                }
            }

            return NOTFOUND;
        }

        template <typename DomainT>
        void prepare(viennamini::Device<DomainT, MatlibT> & device,
                     viennamini::Config                   & config )
        {
          typedef viennamini::Device<DomainT, MatlibT> Device;

          typedef typename Device::Indices Indices;

          //
          // CONTACTS
          //
          Indices& contact_segments = device.get_contact_segments();
          for(typename Indices::iterator iter = contact_segments.begin();
              iter != contact_segments.end(); iter++)
          {

              if(isContactInsulatorInterface(*iter))
              {


                  viennafvm::set_dirichlet_boundary(device.get_domain().segments()[*iter],
                      config.get_contact_value(*iter) + config.get_workfunction(*iter), quantity_potential());

                  std::size_t adjacent_oxide_segment = contactOxideInterfaces[*iter];

                  std::cout << "contact segment " << *iter << " interfaces with insulator segment " << adjacent_oxide_segment
                            << " :: contact potential: " << config.get_contact_value(*iter) <<
                               " workfunction: " << config.get_workfunction(*iter) << std::endl;

                  viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[adjacent_oxide_segment], quantity_electron_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[adjacent_oxide_segment], quantity_hole_density());
                }
              else if(isContactSemiconductorInterface(*iter))
              {
                  std::size_t adjacent_semiconductor_segment = contactSemiconductorInterfaces[*iter];
                  numeric_type ND = device.get_donator(adjacent_semiconductor_segment);
                  numeric_type NA = device.get_acceptor(adjacent_semiconductor_segment);
                  numeric_type builtin_pot = viennamini::built_in_potential(config.temperature(), ND, NA);

                  std::cout << "contact segment " << *iter << " interfaces with semiconductor segment " << adjacent_semiconductor_segment
                            << " :: contact potential: " << config.get_contact_value(*iter) <<
                               " workfunction: " << config.get_workfunction(*iter) <<
                               "builtin-pot: " << builtin_pot << " ND: " << ND << " NA: " << NA << std::endl;

                  viennafvm::set_dirichlet_boundary(
                          device.get_domain().segments()[*iter],  // segment
                          config.get_contact_value(*iter) + config.get_workfunction(*iter) + builtin_pot, // BC value
                          quantity_potential()); // quantity

                  viennafvm::set_dirichlet_boundary(device.get_domain().segments()[*iter], // segment
                          ND, // BC value
                          quantity_electron_density()); // quantity

                  viennafvm::set_dirichlet_boundary(
                          device.get_domain().segments()[*iter], // segment
                          NA, // BC value
                          quantity_hole_density()); // quantity

              }

              viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
              viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
          }

          //
          // OXIDES
          //
          Indices& oxide_segments = device.get_oxide_segments();
          for(typename Indices::iterator iter = oxide_segments.begin();
              iter != oxide_segments.end(); iter++)
          {
              std::cout << "oxide segment " << *iter  << std::endl;
            //TODO should we assign a build-in potential?! (to have a proper initial guess here as well ...)

//            viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
//            viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
          }

          //
          // SEMICONDUCTORS
          //
          Indices& semiconductor_segments = device.get_semiconductor_segments();
          for(typename Indices::iterator iter = semiconductor_segments.begin();
              iter != semiconductor_segments.end(); iter++)
          {
              std::cout << "semiconductor segment " << *iter << std::endl;


            numeric_type builtin_potential_value = viennamini::built_in_potential(
                    config.temperature(), device.get_donator(*iter), device.get_acceptor(*iter));

            viennafvm::set_quantity_region(builtin_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value(builtin_key, device.get_domain().segments()[*iter],
                                          builtin_potential_value);
          }


          std::cout << "setting initial conditions .." << std::endl;
          //
          // Initial conditions (required for nonlinear problems)
          //
          viennafvm::set_initial_guess(device.get_domain(), quantity_potential(),        viennamini::builtin_potential_key());
          viennafvm::set_initial_guess(device.get_domain(), quantity_electron_density(), viennamini::donator_doping_key());
          viennafvm::set_initial_guess(device.get_domain(), quantity_hole_density(),     viennamini::acceptor_doping_key());

          std::cout << "smoothing initial conditions .. " << std::endl;
          //
          // smooth the initial guesses
          //
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_potential(),        viennafvm::arithmetic_mean_smoother());
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_potential(),        viennafvm::arithmetic_mean_smoother());
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_electron_density(), viennafvm::geometric_mean_smoother());
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_electron_density(), viennafvm::geometric_mean_smoother());
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_hole_density(),     viennafvm::geometric_mean_smoother());
          viennafvm::smooth_initial_guess(device.get_domain(), quantity_hole_density(),     viennafvm::geometric_mean_smoother());
        }

        bool isContactInsulatorInterface(std::size_t contact_segment_index)
        {
            return !(contactOxideInterfaces.find(contact_segment_index) == contactOxideInterfaces.end());
        }

        bool isContactSemiconductorInterface(std::size_t contact_segment_index)
        {
            return !(contactSemiconductorInterfaces.find(contact_segment_index) == contactSemiconductorInterfaces.end());
        }


        template <typename DomainT>
        void run(viennamini::Device<DomainT, MatlibT> & device,
                 viennamini::Config                   & config)
        {
//            std::cout << "running simulation" << std::endl;

            typedef typename DomainT::config_type       ConfigType;
            typedef typename ConfigType::cell_tag       CellTag;

            typedef typename viennagrid::result_of::ncell<ConfigType, CellTag::dim>::type   CellType;

            //
            // Specify PDEs:
            //

            viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  permittivity; permittivity.wrap_constant( permittivity_key() );
            viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  donator_doping; donator_doping.wrap_constant( donator_doping_key() );
            viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>  acceptor_doping; acceptor_doping.wrap_constant( acceptor_doping_key() );


            std::cout << "setting parameters .. " << std::endl;
            const double q  = 1.6e-19;
            const double kB = 1.38e-23; // Boltzmann constant
            const double mu_n = 0.1430; // Silicon (m^2/Vs)       // mobility (constant is fine for the moment)
            const double mu_p = 0.046;  // Silicon (m^2/Vs)       // TODO do segment (material) specific assembly, and get mobility from matlib!
            const double T  = config.temperature();
            const double VT = kB * T / q;
            const double D_n  = mu_n * VT;  //diffusion constant
            const double D_p  = mu_p * VT;

            // here is all the fun: specify DD system
            function_symbol_type psi = quantity_potential();         // potential, using id=0
            function_symbol_type n   = quantity_electron_density();  // electron concentration, using id=1
            function_symbol_type p   = quantity_hole_density();      // hole concentration, using id=2

            // Set up the Poisson equation and the two continuity equations
            equation_type poisson_eq = viennamath::make_equation( viennamath::div(permittivity * viennamath::grad(psi)),                     /* = */ q * ((n - donator_doping) - (p - acceptor_doping)));
            equation_type cont_eq_n  = viennamath::make_equation( viennamath::div(D_n * viennamath::grad(n) - mu_n * viennamath::grad(psi) * n), /* = */ 0);
            equation_type cont_eq_p  = viennamath::make_equation( viennamath::div(D_p * viennamath::grad(p) + mu_p * viennamath::grad(psi) * p), /* = */ 0);

            // Specify the PDE system:
            viennafvm::linear_pde_system<> pde_system;
            pde_system.add_pde(poisson_eq, psi); // equation and associated quantity
            pde_system.add_pde(cont_eq_n,  n);   // equation and associated quantity
            pde_system.add_pde(cont_eq_p,  p);    // equation and associated quantity

            pde_system.option(0).damping_term( (n + p) * (-q / VT) );
            pde_system.option(1).geometric_update(true);
            pde_system.option(2).geometric_update(true);

            pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running


            //
            // Create PDE solver instance and run the solver:
            //

            // configure the DD solver
            std::cout << "damening: " << config.dampening() << std::endl;
            std::cout << "linear breaktol: " << config.linear_breaktol() << std::endl;
            std::cout << "linear iterations: " << config.linear_iterations() << std::endl;
            std::cout << "nonlinear iterations: " << config.nonlinear_iterations() << std::endl;
            pde_solver.set_damping(config.dampening());
            pde_solver.set_linear_breaktol(config.linear_breaktol());
            pde_solver.set_linear_iterations(config.linear_iterations());
            pde_solver.set_nonlinear_iterations(config.nonlinear_iterations());
            pde_solver.set_nonlinear_breaktol(config.nonlinear_breaktol());
            std::cout << "starting simulatoin " << std::endl;

            // run the simulation
            pde_solver(pde_system, device.get_domain());
        }

    public:
        function_symbol_type quantity_potential()        const { return function_symbol_type(0); }
        function_symbol_type quantity_electron_density() const { return function_symbol_type(1); }
        function_symbol_type quantity_hole_density()     const { return function_symbol_type(2); }

        vector_type const& result() { return pde_solver.result(); }

    private:
        viennafvm::pde_solver<> pde_solver;
        vector_type             result_;
        MatlibT&                matlib;

        viennamini::builtin_potential_key  builtin_key;


        typedef std::map<std::size_t, std::size_t>  IndexMap;
        IndexMap contactSemiconductorInterfaces;
        IndexMap contactOxideInterfaces;

    };
}

#endif 

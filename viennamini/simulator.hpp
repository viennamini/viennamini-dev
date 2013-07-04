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

// Boost includes:
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_sparse.hpp>

// ViennaMini includes:
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


    template<typename DeviceT, typename MatlibT>
    class simulator
    {
      public:
        typedef double                                                                          Numeric;
        typedef boost::numeric::ublas::vector<double>                                           Vector;
        typedef typename DeviceT::Indices                                                       Indices;
        typedef typename DeviceT::Domain                                                        Domain;

        typedef typename Domain::config_type                                                    Config;
        typedef typename Config::cell_tag                                                       CellTag;
        typedef typename viennagrid::result_of::ncell<Config, CellTag::dim>::type               CellType;
        typedef typename viennagrid::result_of::segment<Config>::type                           Segment;
        typedef typename viennagrid::result_of::ncell_range<Segment, CellTag::dim-1>::type      FacetRange;
        typedef typename viennagrid::result_of::iterator<FacetRange>::type                      FacetIterator;
        typedef typename viennagrid::result_of::ncell_range<Domain, CellTag::dim>::type         CellContainer;
        typedef typename viennagrid::result_of::iterator<CellContainer>::type                   CellIterator;

        typedef viennamath::function_symbol                                                     FunctionSymbol;
        typedef viennamath::equation                                                            Equation;

        typedef viennafvm::pde_solver<>                                                         PDESolver;
        typedef viennafvm::linear_pde_system<>                                                  PDESystem;
        
        typedef viennafvm::boundary_key                                                         BoundaryKey;
        typedef viennafvm::current_iterate_key                                                  IterateKey;
        typedef viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>           Quantity;

        /** 
            @brief C'tor activates the various models in the linear pde system and 
            initializes objects
        */
        simulator(DeviceT& device, MatlibT& matlib, viennamini::config& config) : 
          device(device), matlib(matlib), config(config)
        {
          eps.wrap_constant( eps_key );
          mu_n.wrap_constant( mu_n_key );
          mu_p.wrap_constant( mu_p_key );
          ND.wrap_constant( ND_key );
          NA.wrap_constant( NA_key );
          

          // check the config object, which model is active. add each active 
          // model to the linear pde system ...
          //
          if(config.has_drift_diffusion())
            add_drift_diffusion();
        }

        /** 
            @brief Public simulator 'execute' function. Performs all required
            step for conducting the device simulation. Requires the segments 
            of the 'device' to be identified as contact/oxide/semiconductor.
            Also a doping is required, which will be retrieved from the device
            during the preparations.
        */
        void operator()()
        {
            // detect contact-semiconductor and contact-oxide interfaces
            //
            this->detect_interfaces();

            // finalize the device setup
            //
            this->prepare();

            // write doping and initial guesses (including boundary conditions) to
            // vtk files for analysis
            //
            this->write_device_doping();
            this->write_device_initial_guesses();

            // run the simulation
            //
            this->run();
        }

        /** 
            @brief Writes the doping phi,n,p to a vtk file
        */
        void write_device_doping()
        {

            viennagrid::io::vtk_writer<Domain> my_vtk_writer;
            viennagrid::io::add_scalar_data_on_cells<viennamini::donator_doping_key,   double, Domain>(my_vtk_writer, viennamini::donator_doping_key(),   "donators");
            viennagrid::io::add_scalar_data_on_cells<viennamini::acceptor_doping_key,  double, Domain>(my_vtk_writer, viennamini::acceptor_doping_key(),  "acceptors");
            my_vtk_writer(device.get_domain(), "viennamini_doping");
        }

        /** 
            @brief Writes the initial guess distributions phi,n,p to a vtk file
        */
        void write_device_initial_guesses()
        {
            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_pot(quantity_potential().id());
            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_n(quantity_electron_density().id());
            viennamini::initial_guess_accessor<BoundaryKey, IterateKey>     init_guess_p(quantity_hole_density().id());

            CellContainer const& cells = viennagrid::ncells<CellTag::dim>(device.get_domain());
            for(CellIterator cit = cells.begin(); cit != cells.end(); cit++)
            {
                viennadata::access<std::string, double>("initial_pot")(*cit) = init_guess_pot(*cit);
                viennadata::access<std::string, double>("initial_n")(*cit) = init_guess_n(*cit);
                viennadata::access<std::string, double>("initial_p")(*cit) = init_guess_p(*cit);
            }

            viennagrid::io::vtk_writer<Domain> initial_writer;
            viennagrid::io::add_scalar_data_on_cells<std::string,double, Domain>(initial_writer, "initial_pot", "initial_pot");
            viennagrid::io::add_scalar_data_on_cells<std::string,double, Domain>(initial_writer, "initial_n", "initial_n");
            viennagrid::io::add_scalar_data_on_cells<std::string,double, Domain>(initial_writer, "initial_p", "initial_p");
            initial_writer(device.get_domain(), "viennamini_initial_guesses");
        }

    private:

        /** 
            @brief Method identifies metal-semiconductor/oxide interfaces
        */
        void detect_interfaces()
        {
            Indices& contact_segments       = device.get_contact_segments();
            Indices& oxide_segments         = device.get_oxide_segments();
            Indices& semiconductor_segments = device.get_semiconductor_segments();

            // traverse only contact segments
            // for each contact segment, determine whether it shares an interface with an oxide or a semiconductor
            //
            for(typename Indices::iterator cs_it = contact_segments.begin();
              cs_it != contact_segments.end(); cs_it++)
            {
                Segment& current_contact_segment = device.get_domain().segments()[*cs_it];

                int adjacent_semiconduct_segment_id = find_adjacent_segment(current_contact_segment, semiconductor_segments);
                if(adjacent_semiconduct_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Semiconductor segment #" << adjacent_semiconduct_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactSemiconductorInterfaces[*cs_it] = adjacent_semiconduct_segment_id;
                }
                // if it's not a contact-semiconductor interface -> try a contact-insulator interface 
                int adjacent_oxide_segment_id = find_adjacent_segment(current_contact_segment, oxide_segments);
                if(adjacent_oxide_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Oxide segment #" << adjacent_oxide_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactOxideInterfaces[*cs_it] = adjacent_oxide_segment_id;
                }
            }
        }

        /** 
            @brief Method identifies for a given segment under test whether it 
            shares an interface with a reference contact segment
        */
        template <typename SegmentType, typename IndicesT>
        int find_adjacent_segment(SegmentType                 & current_contact_segment,
                                  IndicesT                    & segments_under_test)
        {
            FacetRange   facets                  = viennagrid::ncells<CellTag::dim-1>(current_contact_segment);

            // segments under test: these are either all oxide or semiconductor segments
            //
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

        /** 
            @brief Perform final steps required for the device simulation:
            1. assign dirichlet boundary conditions
            2. disable obsolete quantities
            3. assign initial guesses
            4. smooth initial guesses
        */
        void prepare()
        {
        #ifdef VIENNAMINI_DEBUG
           std::cout << "* finalizing device segments:" << std::endl;
        #endif

          //
          // CONTACTS
          //
          Indices& contact_segments = device.get_contact_segments();
          for(typename Indices::iterator iter = contact_segments.begin();
              iter != contact_segments.end(); iter++)
          {

              // deactivate the permittivity and the builtin potential for a contact
              //
              viennafvm::set_quantity_region(eps_key,     device.get_domain().segments()[*iter], false);
              viennafvm::set_quantity_region(builtin_key, device.get_domain().segments()[*iter], false);
              
              viennafvm::set_quantity_region(mu_n_key, device.get_domain().segments()[*iter], false);
              viennafvm::set_quantity_region(mu_p_key, device.get_domain().segments()[*iter], false);

              if(isContactInsulatorInterface(*iter))
              {
              #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : contact-to-insulator interface" << std::endl;
              #endif
                  // According to [MB] for a Contact-Insulator interface, it doesn't make sense to use 
                  // a builtin-pot, as there is no adjacent semiconductor segment. adding 'workfunction' instead, 
                  // which can be set by the user
                  // [NOTE] as there is no adjacent semiconductor segment, we must not set a electron/hole BC at 
                  // this contact 
                  //
                  viennafvm::set_dirichlet_boundary(device.get_domain().segments()[*iter],
                      config.get_contact_value(*iter) + config.get_workfunction(*iter), quantity_potential());

                  std::size_t adjacent_oxide_segment = contactOxideInterfaces[*iter];

//                  std::cout << "contact segment " << *iter << " interfaces with insulator segment " << adjacent_oxide_segment
//                            << " :: contact potential: " << config.get_contact_value(*iter) <<
//                               " workfunction: " << config.get_workfunction(*iter) << std::endl;

                  // delete obsolete quantities for the current contact and the adjacent oxide segments
                  // in both, electrons and holes don't make sense.
                  //
                  viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[adjacent_oxide_segment], quantity_electron_density());
                  viennafvm::disable_quantity(device.get_domain().segments()[adjacent_oxide_segment], quantity_hole_density());
                }
              else if(isContactSemiconductorInterface(*iter))
              {
              #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : contact-to-semiconductor interface" << std::endl;
              #endif
              
                  // retrieve the doping values of the adjacent semiconductor segment and 
                  // compute the initial guess
                  std::size_t adjacent_semiconductor_segment = contactSemiconductorInterfaces[*iter];
                  Numeric ND = device.get_donator(adjacent_semiconductor_segment);
                  Numeric NA = device.get_acceptor(adjacent_semiconductor_segment);
                  Numeric builtin_pot = viennamini::built_in_potential(config.temperature(), ND, NA);

//                  std::cout << "contact segment " << *iter << " interfaces with semiconductor segment " << adjacent_semiconductor_segment
//                            << " :: contact potential: " << config.get_contact_value(*iter) <<
//                               " workfunction: " << config.get_workfunction(*iter) <<
//                               "builtin-pot: " << builtin_pot << " ND: " << ND << " NA: " << NA << std::endl;

                  // aside of the contact potential, add the builtin-pot and the workfunction (0 by default ..)
                  //                  
                  viennafvm::set_dirichlet_boundary(
                          device.get_domain().segments()[*iter],  // segment
                          config.get_contact_value(*iter) + config.get_workfunction(*iter) + builtin_pot, // BC value
                          quantity_potential()); // quantity

                  // as this contact is a contact-semiconductor interface, we have to 
                  // provide BCs for the electrons and holes as well
                  //
                  viennafvm::set_dirichlet_boundary(device.get_domain().segments()[*iter], // segment
                          ND, // BC value
                          quantity_electron_density()); // quantity

                  viennafvm::set_dirichlet_boundary(
                          device.get_domain().segments()[*iter], // segment
                          NA, // BC value
                          quantity_hole_density()); // quantity

                // for a contact, the following quantities don't make any sense
                //
                viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
                viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());

              }
          }

          //
          // OXIDES
          //
          Indices& oxide_segments = device.get_oxide_segments();
          for(typename Indices::iterator iter = oxide_segments.begin();
              iter != oxide_segments.end(); iter++)
          {
          #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : oxide" << std::endl;
          #endif
          
            viennafvm::set_quantity_region(eps_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (eps_key, device.get_domain().segments()[*iter],
                                           matlib.getParameterValue(
                                             device.get_segment_materials()[*iter], "permittivity") * viennamini::eps0::val()); 

            viennafvm::set_quantity_region(mu_n_key, device.get_domain().segments()[*iter], false);
            viennafvm::set_quantity_region(mu_p_key, device.get_domain().segments()[*iter], false);

            viennafvm::set_quantity_region(builtin_key, device.get_domain().segments()[*iter], false);

            // disable the electron and hole quantities
            //
            viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_electron_density());
            viennafvm::disable_quantity(device.get_domain().segments()[*iter], quantity_hole_density());
          }

          //
          // SEMICONDUCTORS
          //
          Indices& semiconductor_segments = device.get_semiconductor_segments();
          for(typename Indices::iterator iter = semiconductor_segments.begin();
              iter != semiconductor_segments.end(); iter++)
          {
          #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : semiconductor" << std::endl;
          #endif

            viennafvm::set_quantity_region(eps_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (eps_key, device.get_domain().segments()[*iter],
                                           matlib.getParameterValue(
                                             device.get_segment_materials()[*iter], "permittivity") * viennamini::eps0::val()); 
                                             
            viennafvm::set_quantity_region(mu_n_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (mu_n_key, device.get_domain().segments()[*iter], 1.0);// TODO

            viennafvm::set_quantity_region(mu_p_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (mu_p_key, device.get_domain().segments()[*iter], 1.0);// TODO
            
            viennafvm::set_quantity_region(ND_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (ND_key, device.get_domain().segments()[*iter], device.get_donator(*iter));

            viennafvm::set_quantity_region(NA_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value (NA_key, device.get_domain().segments()[*iter], device.get_acceptor(*iter));


            // within the semiconductor segments, we have to prepare an initial guess quantity for the potential distribution
            // we shall use the builtin-pot here ..
            // [NOTE] I have pimped the builtin-pot implementation, with respect to UT
            //
            Numeric builtin_potential_value = viennamini::built_in_potential(
                    config.temperature(), device.get_donator(*iter), device.get_acceptor(*iter));

            viennafvm::set_quantity_region(builtin_key, device.get_domain().segments()[*iter], true);
            viennafvm::set_quantity_value(builtin_key, device.get_domain().segments()[*iter],
                                          builtin_potential_value);
          }

      #ifdef VIENNAMINI_DEBUG
          std::cout << "* setting initial conditions .." << std::endl;
      #endif
          //
          // Initial conditions (required for nonlinear problems)
          // we use the doping for the n, p - initial guesses
          //
          viennafvm::set_initial_guess(device.get_domain(), quantity_potential(),        viennamini::builtin_potential_key());
          viennafvm::set_initial_guess(device.get_domain(), quantity_electron_density(), viennamini::donator_doping_key());
          viennafvm::set_initial_guess(device.get_domain(), quantity_hole_density(),     viennamini::acceptor_doping_key());

      #ifdef VIENNAMINI_DEBUG
          std::cout << "* smoothing initial conditions " << config. initial_guess_smoothing_iterations() << " times " << std::endl;
      #endif
          //
          // smooth the initial guesses
          // we can set the number of smoothing iterations via the config object
          //
          for(int i = 0; i < config.initial_guess_smoothing_iterations(); i++)
          {
            viennafvm::smooth_initial_guess(device.get_domain(), quantity_potential(),        viennafvm::arithmetic_mean_smoother());
            viennafvm::smooth_initial_guess(device.get_domain(), quantity_electron_density(), viennafvm::geometric_mean_smoother());
            viennafvm::smooth_initial_guess(device.get_domain(), quantity_hole_density(),     viennafvm::geometric_mean_smoother());
         
          }
        }

        /** 
            @brief Test whether the contact segment under test shares an interface with an insulator
        */
        bool isContactInsulatorInterface(std::size_t contact_segment_index)
        {
            return !(contactOxideInterfaces.find(contact_segment_index) == contactOxideInterfaces.end());
        }

        /** 
            @brief Test whether the contact segment under test shares an interface with a semiconductor
        */
        bool isContactSemiconductorInterface(std::size_t contact_segment_index)
        {
            return !(contactSemiconductorInterfaces.find(contact_segment_index) == contactSemiconductorInterfaces.end());
        }

        void add_drift_diffusion()
        {
            const double q  = viennamini::q::val();
            const double kB = viennamini::kB::val(); 
            
            
            // [NOTE] Instead of '1', I am using the values for silicon at 300 K. 
            // Also I am using different mobilities for electrons/holes
            // We need a ViennaModels/ViennaMaterials approach here. .. obviously
            //
//            const double mu_n = 0.1430; // Silicon (m^2/Vs)       // mobility (constant is fine for the moment)
//            const double mu_p = 0.046;  // Silicon (m^2/Vs)       // TODO do segment (material) specific assembly, and get mobility from matlib!
            const double T  = config.temperature();
            viennamath::expr VT = kB * T / q;
 
            // here is all the fun: specify DD system
            FunctionSymbol psi = quantity_potential();         // potential, using id=0
            FunctionSymbol n   = quantity_electron_density();  // electron concentration, using id=1
            FunctionSymbol p   = quantity_hole_density();      // hole concentration, using id=2

            // Set up the Poisson equation and the two continuity equations
            Equation poisson_eq = viennamath::make_equation( viennamath::div(eps  * viennamath::grad(psi)),                                       /* = */ q * ((n - ND) - (p - NA)));
            Equation cont_eq_n  = viennamath::make_equation( viennamath::div(mu_n * VT * viennamath::grad(n) - mu_n * viennamath::grad(psi) * n), /* = */ 0);
            Equation cont_eq_p  = viennamath::make_equation( viennamath::div(mu_p * VT * viennamath::grad(p) + mu_p * viennamath::grad(psi) * p), /* = */ 0);

            // Specify the PDE system:
            pde_system.add_pde(poisson_eq, psi); // equation and associated quantity
            pde_system.add_pde(cont_eq_n,  n);   // equation and associated quantity
            pde_system.add_pde(cont_eq_p,  p);   // equation and associated quantity

            pde_system.option(0).damping_term( (n + p) * (-q / VT) );
            pde_system.option(1).geometric_update(true);
            pde_system.option(2).geometric_update(true);

            pde_system.is_linear(false); // temporary solution up until automatic nonlinearity detection is running
        }

        /** 
            @brief Perform the device simulation. The device has been assigned an initial guess
            and boundary conditions at this point.
        */
        void run()
        {

            // configure the DD solver
            pde_solver.set_damping(config.damping());
            pde_solver.set_linear_breaktol(config.linear_breaktol());
            pde_solver.set_linear_iterations(config.linear_iterations());
            pde_solver.set_nonlinear_iterations(config.nonlinear_iterations());
            pde_solver.set_nonlinear_breaktol(config.nonlinear_breaktol());

//            std::cout << "starting simulatoin " << std::endl;

        #ifdef VIENNAMINI_DEBUG
            std::cout << "* starting simulation .. " << std::endl;
        #endif
            // run the simulation
            pde_solver(pde_system, device.get_domain());
        }

    public:
        FunctionSymbol quantity_potential()        const { return FunctionSymbol(0); }
        FunctionSymbol quantity_electron_density() const { return FunctionSymbol(1); }
        FunctionSymbol quantity_hole_density()     const { return FunctionSymbol(2); }

        Vector const& result() { return pde_solver.result(); }

    private:
        DeviceT&                device;
        MatlibT&                matlib;
        viennamini::config&     config;

        PDESystem               pde_system;
        PDESolver               pde_solver;

        typedef std::map<std::size_t, std::size_t>  IndexMap;
        IndexMap contactSemiconductorInterfaces;
        IndexMap contactOxideInterfaces;

        viennamini::permittivity_key       eps_key;
        viennamini::builtin_potential_key  builtin_key;
        viennamini::donator_doping_key     ND_key;
        viennamini::acceptor_doping_key    NA_key;
        viennamini::mobility_electrons_key mu_n_key;
        viennamini::mobility_holes_key     mu_p_key;

        Quantity  eps;   
        Quantity  ND;  
        Quantity  NA;
        Quantity  mu_n;
        Quantity  mu_p;
    };
}

#endif 

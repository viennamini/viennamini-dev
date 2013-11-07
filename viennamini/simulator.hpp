#ifndef VIENNAMINI_SIMULATOR_HPP
#define VIENNAMINI_SIMULATOR_HPP

/* =======================================================================
   Copyright (c) 2011, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Karl Rupp                          rupp@iue.tuwien.ac.at
               Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the ViennaFVM base directory
======================================================================= */

/* Come up with a simulator object similar to ViennaSHE.
   Configuration should happen in a similar manner, allowing for the selection of predefined models (DD, Hydro, ev. ET)
*/

#ifndef NDEBUG
  #define NDEBUG
#endif

// ViennaFVM includes:
#ifdef VIENNAMINI_DEBUG
  #define VIENNAFVM_VERBOSE
#endif
#include "viennafvm/forwards.h"
#include "viennafvm/linear_assembler.hpp"
#include "viennafvm/io/vtk_writer.hpp"
#include "viennafvm/boundary.hpp"
#include "viennafvm/pde_solver.hpp"
#include "viennafvm/initial_guess.hpp"
#ifdef VIENNACL_WITH_OPENCL
#include "viennafvm/viennacl_support.hpp"
#endif

// ViennaGrid includes:
#include "viennagrid/forwards.hpp"
#include "viennagrid/config/default_configs.hpp"
#include "viennagrid/io/netgen_reader.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/interface.hpp"
#include "viennagrid/algorithm/voronoi.hpp"
#include "viennagrid/algorithm/scale.hpp"

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
#include "viennamini/result_accessor.hpp"

namespace viennamini
{
    static int NOTFOUND = -1;


    template<typename DeviceT, typename MatlibT>
    class simulator
    {
      public:
        typedef DeviceT                                                                         DeviceType;
        typedef MatlibT                                                                         MatlibType;
        typedef typename DeviceT::numeric_type                                                  NumericType;
        typedef typename DeviceT::indices_type                                                  IndicesType;
        typedef typename DeviceT::mesh_type                                                     MeshType;
        typedef typename DeviceT::storage_type                                                  StorageType;
        typedef typename DeviceT::segmentation_type                                             SegmentationType;
        typedef typename DeviceT::segment_type                                                  SegmentType;

        typedef typename viennagrid::result_of::cell_tag<MeshType>::type                        CellTagType;
        typedef typename viennagrid::result_of::facet_tag<MeshType>::type                       FacetTagType;
        typedef typename viennagrid::result_of::cell<MeshType>::type                            CellType;
        typedef typename viennagrid::result_of::facet<MeshType>::type                           FacetType;

        typedef typename viennagrid::result_of::cell_range<MeshType>::type                      CellRangeType;
        typedef typename viennagrid::result_of::iterator<CellRangeType>::type                   CellIteratorType;

        typedef viennamath::function_symbol                                                     FunctionSymbolType;
        typedef viennamath::equation                                                            EquationType;

        typedef viennafvm::linsolv::viennacl                                                    LinerSolverType;
        typedef viennafvm::pde_solver<>                                                         PDESolverType;
        typedef viennafvm::linear_pde_system<>                                                  PDESystemType;
        typedef viennafvm::boundary_key                                                         BoundaryKeyType;
        typedef viennafvm::current_iterate_key                                                  IterateKeyType;
        typedef viennafvm::ncell_quantity<CellType, viennamath::expr::interface_type>           QuantityType;

        typedef boost::numeric::ublas::vector<NumericType>                                      VectorType;
        typedef std::map<std::size_t, std::size_t>                                              IndexMapType;


        /**
            @brief C'tor activates the various models in the linear pde system and
            initializes objects
        */
        simulator(DeviceT& device, MatlibT& matlib, viennamini::config& config) :
          device_(device), matlib_(matlib), config_(config)
        {
          eps_.wrap_constant ( device_.storage(), eps_key_  );
          mu_n_.wrap_constant( device_.storage(), mu_n_key_ );
          mu_p_.wrap_constant( device_.storage(), mu_p_key_ );
          ND_.wrap_constant  ( device_.storage(), ND_key_   );
          NA_.wrap_constant  ( device_.storage(), NA_key_   );
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
            typedef typename viennadata::result_of::accessor<StorageType, viennamini::donator_doping_key, NumericType, CellType>::type DonatorAccessorType;
            typedef typename viennadata::result_of::accessor<StorageType, viennamini::acceptor_doping_key, NumericType, CellType>::type AcceptorAccessorType;

            DonatorAccessorType  donator_acc  = viennadata::make_accessor(device_.storage(), viennamini::donator_doping_key());
            AcceptorAccessorType acceptor_acc = viennadata::make_accessor(device_.storage(), viennamini::acceptor_doping_key());

            viennagrid::io::vtk_writer<MeshType> my_vtk_writer;
            my_vtk_writer.add_scalar_data_on_cells( donator_acc , "donators" );
            my_vtk_writer.add_scalar_data_on_cells( acceptor_acc , "acceptors" );
            my_vtk_writer(device_.mesh(), device_.segments(), "viennamini_doping");
        }

        /**
            @brief Writes the initial guess distributions phi,n,p to a vtk file
        */
        void write_device_initial_guesses()
        {
            typedef typename viennadata::result_of::accessor<StorageType, BoundaryKeyType, NumericType, CellType>::type  BoundaryAccessorType;
            typedef typename viennadata::result_of::accessor<StorageType, IterateKeyType, NumericType, CellType>::type   InitGuessAccessorType;

            BoundaryAccessorType  bnd_pot_acc  = viennadata::make_accessor(device_.storage(), BoundaryKeyType(quantity_potential().id()));
            InitGuessAccessorType init_pot_acc = viennadata::make_accessor(device_.storage(), IterateKeyType(quantity_potential().id()));

            BoundaryAccessorType  bnd_n_acc  = viennadata::make_accessor(device_.storage(), BoundaryKeyType(quantity_electron_density().id()));
            InitGuessAccessorType init_n_acc = viennadata::make_accessor(device_.storage(), IterateKeyType(quantity_electron_density().id()));

            BoundaryAccessorType  bnd_p_acc  = viennadata::make_accessor(device_.storage(), BoundaryKeyType(quantity_hole_density().id()));
            InitGuessAccessorType init_p_acc = viennadata::make_accessor(device_.storage(), IterateKeyType(quantity_hole_density().id()));

            viennagrid::io::vtk_writer<MeshType> bnd_vtk_writer;
            bnd_vtk_writer.add_scalar_data_on_cells( bnd_pot_acc , "potential" );
            bnd_vtk_writer.add_scalar_data_on_cells( bnd_n_acc ,   "electrons" );
            bnd_vtk_writer.add_scalar_data_on_cells( bnd_p_acc ,   "holes" );
            bnd_vtk_writer(device_.mesh(), device_.segments(), "viennamini_boundary_conditions");

            viennagrid::io::vtk_writer<MeshType> init_vtk_writer;
            init_vtk_writer.add_scalar_data_on_cells( init_pot_acc , "potential" );
            init_vtk_writer.add_scalar_data_on_cells( init_n_acc ,   "electrons" );
            init_vtk_writer.add_scalar_data_on_cells( init_p_acc ,   "holes" );
            init_vtk_writer(device_.mesh(), device_.segments(), "viennamini_initial_conditions");
        }

        void write_result(std::string filename = "viennamini_result")
        {
          // Writing all solution variables back to domain.
          //
          std::vector<long> result_ids(3); //TODO: Better way to make potential, electron_density and hole_density accessible
          result_ids[0] = quantity_potential().id();
          result_ids[1] = quantity_electron_density().id();
          result_ids[2] = quantity_hole_density().id();

          viennafvm::io::write_solution_to_VTK_file(result(), filename, device_.mesh(), device_.segments(), device_.storage(), result_ids);
        }


    private:

        /**
            @brief Method identifies metal-semiconductor/oxide interfaces
        */
        void detect_interfaces()
        {
            IndicesType& contact_segments       = device_.contact_segments();
            IndicesType& oxide_segments         = device_.oxide_segments();
            IndicesType& semiconductor_segments = device_.semiconductor_segments();

            // traverse only contact segments
            // for each contact segment, determine whether it shares an interface with an oxide or a semiconductor
            //
            for(typename IndicesType::iterator cs_it = contact_segments.begin();
              cs_it != contact_segments.end(); cs_it++)
            {
                //std::cout << "  * contact-segment " << *cs_it << " : looking for interfaces .." << std::endl;
                SegmentType& current_contact_segment = device_.segment(*cs_it);

                int adjacent_semiconduct_segment_id = find_adjacent_segment(current_contact_segment, semiconductor_segments);
                if(adjacent_semiconduct_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Semiconductor segment #" << adjacent_semiconduct_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactSemiconductorInterfaces_[*cs_it] = adjacent_semiconduct_segment_id;
                }
                // if it's not a contact-semiconductor interface -> try a contact-insulator interface
                int adjacent_oxide_segment_id = find_adjacent_segment(current_contact_segment, oxide_segments);
                if(adjacent_oxide_segment_id != NOTFOUND)
                {
                    //std::cout << "Found neighbour Oxide segment #" << adjacent_oxide_segment_id << " for contact segment #" << *cs_it << std::endl;
                    contactOxideInterfaces_[*cs_it] = adjacent_oxide_segment_id;
                }
            }
        }

        /**
            @brief Method identifies for a given segment under test whether it
            shares an interface with a reference contact segment
        */
        int find_adjacent_segment(SegmentType & current_contact_segment,
                                  IndicesType & segments_under_test)
        {
            typedef typename viennagrid::result_of::const_facet_range<SegmentType>::type            ConstFacetSegmentRangeType;
            typedef typename viennagrid::result_of::iterator<ConstFacetSegmentRangeType>::type      ConstFacetSegmentIteratorType;

            ConstFacetSegmentRangeType const& facets = viennagrid::elements<FacetType>(current_contact_segment);

            // segments under test: these are either all oxide or semiconductor segments
            //
            for(typename IndicesType::iterator sit = segments_under_test.begin();
                sit != segments_under_test.end(); sit++)
            {
                SegmentType& current_segment = device_.segment(*sit);

                for (ConstFacetSegmentIteratorType fit = facets.begin(); fit != facets.end(); ++fit)
                {
                    if (viennagrid::is_interface(current_contact_segment, current_segment, *fit))
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

          StorageType & storage = device_.storage();

          //
          // CONTACTS
          //
          IndicesType& contact_segments = device_.contact_segments();
          for(typename IndicesType::iterator iter = contact_segments.begin();
              iter != contact_segments.end(); iter++)
          {
              // deactivate the permittivity and the builtin potential for a contact
              //
              viennafvm::set_quantity_region(device_.segment(*iter), storage, builtin_key_, false);
              viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_n_key_,    false);
              viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_p_key_,    false);

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
                  viennafvm::set_dirichlet_boundary(
                      device_.segment(*iter),
                      storage,
                      quantity_potential(),
                      config_.contact_value(*iter) + config_.workfunction(*iter)
                      );

                  std::size_t adjacent_oxide_segment = contactOxideInterfaces_[*iter];

//                  std::cout << "contact segment " << *iter << " interfaces with insulator segment " << adjacent_oxide_segment
//                            << " :: contact potential: " << config.get_contact_value(*iter) <<
//                               " workfunction: " << config.get_workfunction(*iter) << std::endl;

                  // a contact segment needs the permittivity as well. we use the 
                  // permittivity from the adjacent segment
                  //
                  viennafvm::set_quantity_region(device_.segment(*iter), storage, eps_key_, true);
                  viennafvm::set_quantity_value (device_.segment(*iter), storage, eps_key_,
                                                 matlib_.getParameterValue(
                                                   device_.segment_materials()[adjacent_oxide_segment], "permittivity") * viennamini::eps0::val());

                  // delete obsolete quantities for the current contact and the adjacent oxide segments
                  // in both, electrons and holes don't make sense.
                  //
                  viennafvm::disable_quantity(device_.segment(*iter),                  storage, quantity_electron_density());
                  viennafvm::disable_quantity(device_.segment(*iter),                  storage, quantity_hole_density());
                  viennafvm::disable_quantity(device_.segment(adjacent_oxide_segment), storage, quantity_electron_density());
                  viennafvm::disable_quantity(device_.segment(adjacent_oxide_segment), storage, quantity_hole_density());
                }
              else if(isContactSemiconductorInterface(*iter))
              {
              #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : contact-to-semiconductor interface" << std::endl;
              #endif

                  // retrieve the doping values of the adjacent semiconductor segment and
                  // compute the initial guess
                  std::size_t adjacent_semiconductor_segment = contactSemiconductorInterfaces_[*iter];
                  NumericType ND_value = device_.donator(adjacent_semiconductor_segment);
                  NumericType NA_value = device_.acceptor(adjacent_semiconductor_segment);
                  NumericType builtin_pot = viennamini::built_in_potential(config_.temperature(), ND_value, NA_value);

                  // a contact segment needs the permittivity as well. we use the 
                  // permittivity from the adjacent segment
                  //
                  viennafvm::set_quantity_region(device_.segment(*iter), storage, eps_key_, true);
                  viennafvm::set_quantity_value (device_.segment(*iter), storage, eps_key_,
                                                 matlib_.getParameterValue(
                                                   device_.segment_materials()[adjacent_semiconductor_segment], "permittivity") * viennamini::eps0::val());


//                  std::cout << "contact segment " << *iter << " interfaces with semiconductor segment " << adjacent_semiconductor_segment
//                            << " :: contact potential: " << config.get_contact_value(*iter) <<
//                               " workfunction: " << config.get_workfunction(*iter) <<
//                               " builtin-pot: " << builtin_pot << " ND: " << ND << " NA: " << NA << std::endl;

                  // aside of the contact potential, add the builtin-pot and the workfunction (0 by default ..)
                  //
                  viennafvm::set_dirichlet_boundary(
                          device_.segment(*iter),  // segment
                          storage,
                          quantity_potential(),
                          config_.contact_value(*iter) + config_.workfunction(*iter) + builtin_pot // BC value
                          );

                  // as this contact is a contact-semiconductor interface, we have to
                  // provide BCs for the electrons and holes as well
                  //
                  viennafvm::set_dirichlet_boundary(
                          device_.segment(*iter), // segment
                          storage,
                          quantity_electron_density(),
                          ND_value // BC value
                          );

                  viennafvm::set_dirichlet_boundary(
                          device_.segment(*iter), // segment
                          storage,
                          quantity_hole_density(),
                          NA_value // BC value
                          );

                // for a contact, the following quantities don't make any sense
                //
                viennafvm::disable_quantity(device_.segment(*iter), storage, quantity_electron_density());
                viennafvm::disable_quantity(device_.segment(*iter), storage, quantity_hole_density());

              }
          }

          //
          // OXIDES
          //
          IndicesType& oxide_segments = device_.oxide_segments();
          for(typename IndicesType::iterator iter = oxide_segments.begin();
              iter != oxide_segments.end(); iter++)
          {
          #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : oxide" << std::endl;
          #endif

            viennafvm::set_quantity_region(device_.segment(*iter), storage, eps_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, eps_key_,
                                           matlib_.getParameterValue(
                                             device_.segment_materials()[*iter], "permittivity") * viennamini::eps0::val());

            viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_n_key_, false);
            viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_p_key_, false);
            viennafvm::set_quantity_region(device_.segment(*iter), storage, builtin_key_, false);

            // disable the electron and hole quantities
            //
            viennafvm::disable_quantity(device_.segment(*iter), storage, quantity_electron_density());
            viennafvm::disable_quantity(device_.segment(*iter), storage, quantity_hole_density());
          }

          //
          // SEMICONDUCTORS
          //
          IndicesType& semiconductor_segments = device_.semiconductor_segments();
          for(typename IndicesType::iterator iter = semiconductor_segments.begin();
              iter != semiconductor_segments.end(); iter++)
          {
          #ifdef VIENNAMINI_DEBUG
                 std::cout << "  * segment " << *iter << " : semiconductor" << std::endl;
          #endif

            viennafvm::set_quantity_region(device_.segment(*iter), storage, eps_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, eps_key_,
                                           matlib_.getParameterValue(
                                             device_.segment_materials()[*iter], "permittivity") * viennamini::eps0::val());

            viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_n_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, mu_n_key_, 0.1430);// TODO

            viennafvm::set_quantity_region(device_.segment(*iter), storage, mu_p_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, mu_p_key_, 0.046);// TODO

            viennafvm::set_quantity_region(device_.segment(*iter), storage, ND_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, ND_key_, device_.donator(*iter));

            viennafvm::set_quantity_region(device_.segment(*iter), storage, NA_key_, true);
            viennafvm::set_quantity_value (device_.segment(*iter), storage, NA_key_, device_.acceptor(*iter));


            // within the semiconductor segments, we have to prepare an initial guess quantity for the potential distribution
            // we shall use the builtin-pot here ..
            // [NOTE] I have pimped the builtin-pot implementation, with respect to UT
            //
            NumericType builtin_potential_value = viennamini::built_in_potential(
                    config_.temperature(), device_.donator(*iter), device_.acceptor(*iter));

            viennafvm::set_quantity_region(device_.segment(*iter), storage, builtin_key_, true);
            viennafvm::set_quantity_value(device_.segment(*iter), storage, builtin_key_, builtin_potential_value);
          }

      #ifdef VIENNAMINI_DEBUG
          std::cout << "* setting initial conditions .." << std::endl;
      #endif
          //
          // Initial conditions (required for nonlinear problems)
          // we use the doping for the n, p - initial guesses
          //
          viennafvm::set_initial_guess(device_.mesh(), storage, quantity_potential(),        viennamini::builtin_potential_key());
          viennafvm::set_initial_guess(device_.mesh(), storage, quantity_electron_density(), viennamini::donator_doping_key());
          viennafvm::set_initial_guess(device_.mesh(), storage, quantity_hole_density(),     viennamini::acceptor_doping_key());

      #ifdef VIENNAMINI_DEBUG
          std::cout << "* smoothing initial conditions " << config_.initial_guess_smoothing_iterations() << " times " << std::endl;
      #endif
          //
          // smooth the initial guesses
          // we can set the number of smoothing iterations via the config object
          //
          for(int i = 0; i < config_.initial_guess_smoothing_iterations(); i++)
          {
            viennafvm::smooth_initial_guess(device_.mesh(), storage,
                                            viennafvm::arithmetic_mean_smoother(), quantity_potential());

            viennafvm::smooth_initial_guess(device_.mesh(), storage,
                                            viennafvm::geometric_mean_smoother(), quantity_electron_density());

            viennafvm::smooth_initial_guess(device_.mesh(), storage,
                                            viennafvm::geometric_mean_smoother(), quantity_hole_density());
          }
        }

        /**
            @brief Test whether the contact segment under test shares an interface with an insulator
        */
        bool isContactInsulatorInterface(std::size_t contact_segment_index)
        {
            return !(contactOxideInterfaces_.find(contact_segment_index) == contactOxideInterfaces_.end());
        }

        /**
            @brief Test whether the contact segment under test shares an interface with a semiconductor
        */
        bool isContactSemiconductorInterface(std::size_t contact_segment_index)
        {
            return !(contactSemiconductorInterfaces_.find(contact_segment_index) == contactSemiconductorInterfaces_.end());
        }

        void add_drift_diffusion()
        {
            const NumericType q  = viennamini::q::val();
            const NumericType kB = viennamini::kB::val();


            const NumericType T  = config_.temperature();
            viennamath::expr VT = kB * T / q;

            // here is all the fun: specify DD system
            FunctionSymbolType psi = quantity_potential();         // potential, using id=0
            FunctionSymbolType n   = quantity_electron_density();  // electron concentration, using id=1
            FunctionSymbolType p   = quantity_hole_density();      // hole concentration, using id=2

            // Set up the Poisson equation and the two continuity equations
            EquationType poisson_eq = viennamath::make_equation( viennamath::div(eps_  * viennamath::grad(psi)),                                       /* = */ q * ((n - ND_) - (p - NA_)));
            EquationType cont_eq_n  = viennamath::make_equation( viennamath::div(mu_n_ * VT * viennamath::grad(n) - mu_n_ * viennamath::grad(psi) * n), /* = */ 0);
            EquationType cont_eq_p  = viennamath::make_equation( viennamath::div(mu_p_ * VT * viennamath::grad(p) + mu_p_ * viennamath::grad(psi) * p), /* = */ 0);

            // Specify the PDE system:
            pde_system_.add_pde(poisson_eq, psi); // equation and associated quantity
            pde_system_.add_pde(cont_eq_n,  n);   // equation and associated quantity
            pde_system_.add_pde(cont_eq_p,  p);   // equation and associated quantity

            pde_system_.option(0).damping_term( (n + p) * (-q / VT) );
            pde_system_.option(1).geometric_update(true);
            pde_system_.option(2).geometric_update(true);

            pde_system_.is_linear(false); // temporary solution up until automatic nonlinearity detection is running
        }

        /**
            @brief Perform the device simulation. The device has been assigned an initial guess
            and boundary conditions at this point.
        */
        void run()
        {
          // check the config object, which model is active. add each active
          // model to the linear pde system ...
          //
          if(config_.drift_diffusion_state())
            add_drift_diffusion();
        
          linear_solver_.max_iterations()  = config_.linear_iterations();
          linear_solver_.break_tolerance() = config_.linear_breaktol();
        
          // configure the DD solver
          pde_solver_.set_damping(config_.damping());
          pde_solver_.set_nonlinear_iterations(config_.nonlinear_iterations());
          pde_solver_.set_nonlinear_breaktol(config_.nonlinear_breaktol());

//            std::cout << "starting simulatoin " << std::endl;

      #ifdef VIENNAMINI_DEBUG
          std::cout << "* starting simulation .. " << std::endl;
      #endif
          // run the simulation
          pde_solver_(pde_system_, device_.mesh(), device_.storage(), linear_solver_);
        }

    public:
        FunctionSymbolType quantity_potential()        const { return FunctionSymbolType(0); }
        FunctionSymbolType quantity_electron_density() const { return FunctionSymbolType(1); }
        FunctionSymbolType quantity_hole_density()     const { return FunctionSymbolType(2); }

        VectorType const& result() { return pde_solver_.result(); }

    private:
        DeviceType            & device_;
        MatlibType            & matlib_;
        viennamini::config    & config_;

        PDESystemType           pde_system_;
        PDESolverType           pde_solver_;
        LinerSolverType         linear_solver_;

        IndexMapType contactSemiconductorInterfaces_;
        IndexMapType contactOxideInterfaces_;

        viennamini::permittivity_key       eps_key_;
        viennamini::builtin_potential_key  builtin_key_;
        viennamini::donator_doping_key     ND_key_;
        viennamini::acceptor_doping_key    NA_key_;
        viennamini::mobility_electrons_key mu_n_key_;
        viennamini::mobility_holes_key     mu_p_key_;

        QuantityType  eps_;
        QuantityType  ND_;
        QuantityType  NA_;
        QuantityType  mu_n_;
        QuantityType  mu_p_;
    };
}

#endif

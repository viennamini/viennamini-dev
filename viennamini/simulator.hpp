#ifndef VIENNAMINI_SIMULATOR_HPP
#define VIENNAMINI_SIMULATOR_HPP

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

/* Come up with a simulator object similar to ViennaSHE.
   Configuration should happen in a similar manner, allowing for the selection of predefined models (DD, Hydro, ev. ET)
*/

#ifndef NDEBUG
  #define NDEBUG
#endif

// System includes


// ViennaFVM includes:
#define VIENNAFVM_VERBOSE
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
#include "viennagrid/config/default_configs.hpp"
#include "viennagrid/io/netgen_reader.hpp"
#include "viennagrid/io/vtk_writer.hpp"
#include "viennagrid/algorithm/voronoi.hpp"
#include "viennagrid/algorithm/scale.hpp"

// ViennaMath includes:
#include "viennamath/expression.hpp"

// Boost includes:
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_sparse.hpp>

// ViennaMini includes:
#include "viennamini/forwards.h"
//#include "viennamini/physics.hpp"
//#include "viennamini/constants.hpp"
#include "viennamini/config.hpp"
#include "viennamini/device.hpp"
#include "viennamini/material_library.hpp"
//#include "viennamini/result_accessor.hpp"
#include "viennamini/problem.hpp"

namespace viennamini
{
  class simulator
  {
  public:
    simulator();
    ~simulator();

    void                                 run();

    viennamini::data_storage      const& storage() const;
    viennamini::data_storage           & storage();

    viennamini::device            const& device() const;
    viennamini::device                 & device();

    viennamini::config            const& config() const;
    viennamini::config                 & config();
  
    viennamini::material_library  const& material_library() const;
    viennamini::material_library       & material_library();
  
    void                                 write(std::string const filename);
  
  private:

    viennamini::data_storage      storage_;
    viennamini::device            device_;
    viennamini::config            config_;
    viennamini::material_library  matlib_;

    viennamini::problem*          problem_;
    
    bool storage_changed_;
    bool device_changed_;
    bool config_changed_;
    bool matlib_changed_;
  };
}

#endif


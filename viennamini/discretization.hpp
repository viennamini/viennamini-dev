#ifndef VIENNAMINI_DISCRETIZATION_HPP
#define VIENNAMINI_DISCRETIZATION_HPP

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


namespace viennamini {

class discretization_exception : public std::runtime_error {
public:
  discretization_exception(std::string const & str) : std::runtime_error(str) {}
};

#define VIENNAMINI_DISCRETIZATION(classname) \
public: \
\
  classname(viennamini::device_handle        device,  \
            viennamini::configuration_handle config,  \
            viennamini::stepper_handle       stepper,  \
            std::ostream                   & stream) : \
            viennamini::discretization(device, config, stepper, stream) {} \
 \
  ~classname() {} \
\
  void run_auto() \
  {\
    if(device().is_line1d()) \
    {\
      this->run(device().get_segmesh_line_1d()); \
    }\
    else \
    if(device().is_triangular2d()) \
    {\
      this->run(device().get_segmesh_triangular_2d()); \
    }\
    else \
    if(device().is_tetrahedral3d()) \
    {\
      this->run(device().get_segmesh_tetrahedral_3d()); \
    }\
    else throw discretization_exception("Mesh type not supported!"); \
  }

class discretization
{
public:
  discretization(viennamini::device_handle        device,
                 viennamini::configuration_handle config,
                 viennamini::stepper_handle       stepper,
                 std::ostream                   & stream);

  virtual ~discretization();

  virtual void run_auto() = 0;

  viennamini::device&         device();
  viennamini::device_handle&  device_handle();
  viennamini::configuration&  config();
  viennamini::stepper&        stepper();
  std::ostream&               stream();

private:
  viennamini::device_handle           device_;
  viennamini::configuration_handle    config_;
  viennamini::stepper_handle          stepper_;
  std::ostream                      & stream_;
};

} // viennamini


#endif


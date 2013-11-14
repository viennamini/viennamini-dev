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


#include "viennamini/simulator.hpp"


namespace viennamini
{

template<typename MatlibT>
simulator<MatlibT>::simulator(MatlibT& matlib) : matlib_(matlib) 
{
}

template<typename MatlibT>
void simulator<MatlibT>::operator()(viennamini::device& device, viennamini::config& config)
{

}

} // viennamini

      


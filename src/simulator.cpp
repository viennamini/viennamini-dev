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

simulator::simulator() : device_(storage_), device_changed(true)
{
}

viennamini::data_storage  const& simulator::storage() const
{
  return storage_;
}

viennamini::data_storage       & simulator::storage()
{
  storage_changed = true;
  return storage_;
}

viennamini::device const& simulator::device() const
{
  return device_;
}

viennamini::device      & simulator::device()
{
  device_changed = true; 
  return device_;
}

viennamini::config const& simulator::config() const
{
  return config_;
}

viennamini::config      & simulator::config()
{
  config_changed = true; 
  return config_;
}

void simulator::run()
{
}

} // viennamini



/* =======================================================================
   Copyright (c) 2011-2015, Institute for Microelectronics, TU Wien
   http://www.iue.tuwien.ac.at
                             -----------------
                 ViennaMini - The Vienna Device Simulator
                             -----------------

   authors:    Josef Weinbub                   weinbub@iue.tuwien.ac.at
               (add your name here)

   license:    see file LICENSE in the base directory
======================================================================= */


// ViennaMini includes
//
#include "viennamini/simulator.hpp"
#include "viennamini/configure_simulator.hpp"


int main(int argc, char* argv[])
{
  if(argc != 2)
  {
    std::cout << "Error  - Usage: " << argv[0] << "  configuration_file" << std::endl;
    std::cout << "Aborting .." << std::endl;
    return EXIT_FAILURE;
  }

  // create the simulator
  //
  viennamini::simulator  sim(std::cout);

  // configure the simulator
  //
  viennamini::configure_simulator(sim, argv[1]);

  // execute the simulation
  //
  sim.run();

  return EXIT_SUCCESS;
}

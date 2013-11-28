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


#include "templates/diode_np2d.hpp"
#include "viennamini/simulator.hpp"

int main()
{
  // a device generator can generate several devices, in this case a 2D Capacitor
  //
  std::string material_library_file = "../../examples/materials.xml";
  viennamini::device_template_handle device_generator(new viennamini::diode_np2d(material_library_file));

  // generate the device, i.e., mesh the geometry and assign segment roles
  //
  device_generator->generate();

  // the generator produces a configuration and device object 
  //
  viennamini::config_handle & myconfig = device_generator->config();
  viennamini::device_handle & mydevice = device_generator->device();

  myconfig->write_initial_guess_files()          = false;
  myconfig->write_result_files()                 = true;
  myconfig->damping()                            = 0.5;
  myconfig->linear_breaktol()                    = 1.0E-14;
  myconfig->linear_iterations()                  = 1000;
  myconfig->nonlinear_iterations()               = 100;
  myconfig->nonlinear_breaktol()                 = 1.0E-2;

  // setup a simulator object and link to a material file
  //
  viennamini::simulator   mysim;

  // set the device and config objects, generated by the above template generator
  //
  mysim.set_device(mydevice);
  mysim.set_config(myconfig);

  // set contact potentials
  //
  mysim.stepper().add(device_generator->segment_indices()["Anode"], 0.9, 0.9, 0.0);
  mysim.stepper().write(std::cout);

  // write the simulation results to output files
  //
  mysim.set_output_filename_prefix("diode_np2d_result");  
  
  // perform the simulation
  //
  mysim.run();

  viennamini::csv mycsv = mysim.csv();
  mycsv.write("diode_np2d_characteristics.csv");

  return 0;
}



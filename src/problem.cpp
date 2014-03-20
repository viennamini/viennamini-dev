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


#include "viennamini/problem.hpp"

namespace viennamini 
{

problem::problem(std::ostream& stream) : stream_(stream) {}

problem::~problem() {}

void problem::set_device_handle(viennamini::device_handle new_device)
{
  device_handle_ = new_device;
}

void problem::set_config_handle(viennamini::config_handle new_config)
{
  config_handle_ = new_config;
}

viennamini::device & problem::device()
{
  return *device_handle_;
}

viennamini::config & problem::config()
{
  return *config_handle_;
}

void problem::write(std::string const& filename, std::size_t step_id)
{
//  if(device().is_line1d())
//  {
//    viennafvm::io::write_solution_to_VTK_file(
//      device().get_problem_description_line_1d(step_id).quantities(),
//      filename,
//      device().get_segmesh_line_1d().mesh,
//      device().get_segmesh_line_1d().segmentation);
//  }
//  else
//  if(device().is_triangular2d())
//  {
//    viennafvm::io::write_solution_to_VTK_file(
//      device().get_problem_description_triangular_2d(step_id).quantities(),
//      filename,
//      device().get_segmesh_triangular_2d().mesh,
//      device().get_segmesh_triangular_2d().segmentation);
//  }
//  else
//  if(device().is_tetrahedral3d())
//  {  
//    viennafvm::io::write_solution_to_VTK_file(
//      device().get_problem_description_tetrahedral_3d(step_id).quantities(),
//      filename,
//      device().get_segmesh_tetrahedral_3d().mesh,
//      device().get_segmesh_tetrahedral_3d().segmentation);
//  }
//  else throw device_not_supported_exception("at: problem::write()");
}

viennamini::data_table& problem::data_table() 
{ 
  return data_table_; 
}

std::ostream& problem::stream()
{
  return stream_;
}

} // viennamini


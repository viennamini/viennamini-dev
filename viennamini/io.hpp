#ifndef VIENNAMINI_IO_HPP
#define VIENNAMINI_IO_HPP

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


#include "viennagrid/io/vtk_writer.hpp"

namespace viennamini {
namespace io {

namespace detail {

template<typename MeshT, typename SegmentationT>
void write_vtk_impl(MeshT& mesh, SegmentationT& segmentation, std::string const& filename)
{
  viennagrid::io::vtk_writer<MeshT> vtk_writer;
  vtk_writer(mesh, segmentation, filename);
}

} // detail



void write_vtk(viennamini::device& device, std::string const& filename)
{
  if(device.is_triangular2d())
  {
    viennamini::SegmentationTriangular2DType & segmentation = boost::get<viennamini::SegmentationTriangular2DType>(device.generic_segmentation());
    viennamini::MeshTriangular2DType         & mesh         = boost::get<viennamini::MeshTriangular2DType>        (device.generic_mesh());

    viennamini::io::detail::write_vtk_impl(mesh, segmentation, filename);
  }
  else if(device.is_tetrahedral3d())
  {
    viennamini::SegmentationTetrahedral3DType & segmentation = boost::get<viennamini::SegmentationTetrahedral3DType>(device.generic_segmentation());
    viennamini::MeshTetrahedral3DType         & mesh         = boost::get<viennamini::MeshTetrahedral3DType>        (device.generic_mesh());

    viennamini::io::detail::write_vtk_impl(mesh, segmentation, filename);
  }
}


void read_vtk(viennamini::device& device, std::string const& filename, viennagrid::config::triangular_2d const&)
{
  device.make_triangular2d();
  viennamini::SegmentationTriangular2DType & segmentation = boost::get<viennamini::SegmentationTriangular2DType>(device.generic_segmentation());
  viennamini::MeshTriangular2DType         & mesh         = boost::get<viennamini::MeshTriangular2DType>        (device.generic_mesh());
  try
  {
    viennagrid::io::netgen_reader my_reader;
    my_reader(mesh, segmentation, filename);
  }
  catch (...)
  {
    std::cerr << "VTK File-Reader failed. Aborting program..." << std::endl;
  }
}

void read_vtk(viennamini::device& device, std::string const& filename, viennagrid::config::tetrahedral_3d const&)
{
  device.make_tetrahedral3d();
  viennamini::SegmentationTetrahedral3DType & segmentation = boost::get<viennamini::SegmentationTetrahedral3DType>(device.generic_segmentation());
  viennamini::MeshTetrahedral3DType         & mesh         = boost::get<viennamini::MeshTetrahedral3DType>        (device.generic_mesh());
  try
  {
    viennagrid::io::netgen_reader my_reader;
    my_reader(mesh, segmentation, filename);
  }
  catch (...)
  {
    std::cerr << "VTK File-Reader failed. Aborting program..." << std::endl;
  }
}

} // io
} // viennamini

#endif


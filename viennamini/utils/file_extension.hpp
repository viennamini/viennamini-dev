
#ifndef VIENNAMINI_FILEEXTENSION_HPP
#define VIENNAMINI_FILEEXTENSION_HPP

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

namespace viennamini {


inline bool file_exists(std::string const& filename)
{
   std::ifstream ifile(filename.c_str());
   return ifile;
}

inline std::string file_extension(std::string const& filename)
{
   return filename.substr(filename.rfind(".")+1, filename.size());
}


} // viennamini

#endif


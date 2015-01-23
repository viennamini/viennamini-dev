#ifndef VIENNAMINI_UTILS_STRING_HPP
#define VIENNAMINI_UTILS_STRING_HPP

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

// System includes
//
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <sstream>
#include <vector>
#include <algorithm>

namespace viennamini {

inline std::string& ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

inline std::string& rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}

inline std::string& trim(std::string &s)
{
  return ltrim(rtrim(s));
}

inline void split(std::vector<std::string>& result_container, std::string const& str, char const& delimiter)
{
  std::istringstream buf(str);
  for(std::string token; getline(buf, token, delimiter); )
      result_container.push_back(token);
}

inline void replace_first(std::string & target, std::string const& placeholder, std::string const& replacement)
{
  size_t start_pos = target.find(placeholder);
  if(start_pos == std::string::npos) return;
  target.replace(start_pos, placeholder.length(), replacement);
}

} // viennamini


#endif

#ifndef VIENNAMINI_RESULTACCESSORS_HPP
#define VIENNAMINI_RESULTACCESSORS_HPP

#include "viennafvm/forwards.h"

namespace viennamini {


template<typename ResultVector>
struct result_accessor
{
  typedef typename ResultVector::value_type   value_type;
  typedef viennafvm::mapping_key              mapping_key_type;
  typedef viennafvm::boundary_key             boundary_key_type;

  result_accessor(ResultVector const& result, std::size_t const& id) : result(result), id(id), map_key(id), bnd_key(id) {}

  template<typename CellT>
  value_type operator()(CellT const& cell)
  {
    long cur_index = viennadata::access<mapping_key_type, long>(map_key)(cell);
    // if interior or non-Dirichlet boundary
    if(cur_index > -1)
    {
      return result[cur_index];
    }
    else //use Dirichlet boundary data:
    {
      return viennadata::access<boundary_key_type, double>(bnd_key)(cell);
    }
  }
  
  template<typename CellT>
  value_type operator()(CellT const& cell) const
  {
    long cur_index = viennadata::access<mapping_key_type, long>(map_key)(cell);
    // if interior or non-Dirichlet boundary
    if(cur_index > -1)
    {
      return result[cur_index];
    }
    else //use Dirichlet boundary data:
    {
      return viennadata::access<boundary_key_type, double>(bnd_key)(cell);
    }
  }
  
  ResultVector const&   result;
  std::size_t     id;
  mapping_key_type  map_key;
  boundary_key_type bnd_key;  
};



} // viennamini


#endif


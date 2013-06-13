#ifndef VIENNAMINI_RESULTACCESSORS_HPP
#define VIENNAMINI_RESULTACCESSORS_HPP

#include "viennafvm/forwards.h"

namespace viennamini {


template<typename ResultVector>
struct result_accessor
{
  typedef typename ResultVector::value_type   NumericType;
  typedef viennafvm::mapping_key              MappingKeyType;
  typedef viennafvm::boundary_key             BoundaryKeyType;

  result_accessor(ResultVector const& result, std::size_t const& id) : result(result), id(id), map_key(id), bnd_key(id) {}

  template<typename CellT>
  NumericType operator()(CellT& cell)
  {
    long cur_index = viennadata::access<MappingKeyType, long>(map_key)(cell);
    // if interior or non-Dirichlet boundary
    if(cur_index > -1)
    {
      return result[cur_index];
    }
    else //use Dirichlet boundary data:
    {
      return viennadata::access<BoundaryKeyType, double>(bnd_key)(cell);
    }
  }
  ResultVector const&   result;
  std::size_t     id;
  MappingKeyType  map_key;
  BoundaryKeyType bnd_key;  
};



} // viennamini


#endif


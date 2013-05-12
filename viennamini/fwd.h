#ifndef VIENNAMINI_FORWARDS_H
#define VIENNAMINI_FORWARDS_H

/** Forward declarations */


namespace viennamini
{

  //
  // Defining a bunch of accessor keys for physical quantities.
  // These should be part of a semiconductor application based on ViennaFVM, not of ViennaFVM itself
  // (which is just a generic finite volume solver and agnostic with respect to the actual physics).
  //

  struct permittivity_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(permittivity_key const & other) const { return false; }
  };

  struct builtin_potential_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(builtin_potential_key const & other) const { return false; }
  };

  // N_D
  struct donator_doping_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(donator_doping_key const & other) const { return false; }
  };

  // N_A
  struct acceptor_doping_key
  {
    // Operator< is required for compatibility with std::map
    bool operator<(acceptor_doping_key const & other) const { return false; }
  };

}

#endif

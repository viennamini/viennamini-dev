

namespace viennamini
{


  double built_in_potential(double temperature, double doping_n, double doping_p)
  {
    const double net_doping = doping_n - doping_p;
    const double x = std::abs(net_doping) / (2.0 * 1e16);

    double bpot = 0.026 * std::log(x + std::sqrt( 1.0 + x*x ) );
                                // V_T * arsinh( net_doping/(2 n_i))

    if ( net_doping < 0) //above formula does not yet consider the doping polarity
      bpot *= -1.0;

    return bpot;
  }

}

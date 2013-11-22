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


#include "viennamini/templates/capacitor1d.hpp"
#include "viennamini/simulator.hpp"


//template<typename NumericT>
//NumericT c1(NumericT x)
//{
//  return x;
//}

//template<typename NumericT>
//NumericT c2(NumericT x, NumericT l)
//{
//  return x + l;
//}

//template<typename NumericT>
//NumericT i1(NumericT x, NumericT l, NumericT a1)
//{
//  return x + l * a1;
//}

//template<typename NumericT>
//NumericT i2(NumericT x, NumericT l, NumericT a2)
//{
//  return x + l * a2;
//}


struct parameters
{
  double x;
  double l;
  double v1;
  double v2;
  double a1;
  double a2;
  double d1_epsr;
  double d2_epsr;
  double d3_epsr;
};

void test()
{
  std::string material_library_file = "../../examples/materials.xml";
  viennamini::device_template_handle device_generator(new viennamini::capacitor1d(material_library_file));
  
  typedef viennamini::device_template::point_type PointType;
  device_generator->geometry_properties()["C11"]  = PointType(-0.5);
  device_generator->geometry_properties()["C1"]   = PointType(0.0);
  device_generator->geometry_properties()["I1"]   = PointType(1.0);
  device_generator->geometry_properties()["I2"]   = PointType(2.0);
  device_generator->geometry_properties()["C2"]   = PointType(3.0);
  device_generator->geometry_properties()["C21"]  = PointType(3.5);

  device_generator->generate();
  
  viennamini::config_handle & myconfig = device_generator->config();
  viennamini::device_handle & mydevice = device_generator->device();

  viennamini::simulator   mysim;
  
  mysim.set_device(mydevice);
  mysim.set_config(myconfig);
  
  mysim.run();
}


int main()
{
  std::vector<parameters> myparas;
  
  {
    parameters p;
    p.x  = -0.1E-6;
    p.l  =   10E-6;
    p.v1 = -0.1;
    p.v2 = -0.1;
    p.a1 = 1./3.;
    p.a2 = 2./3.;
    myparas.push_back(p);
  }
  {
    parameters p;
    p.x  = -0.1E-6;
    p.l  =   10E-6;
    p.v1 =  0.4;
    p.v2 = -0.1;
    p.a1 = 1./3.;
    p.a2 = 2./3.;
    myparas.push_back(p);
  }
  {
    parameters p;
    p.x  = -0.1E-6;
    p.l  =   10E-6;
    p.v1 =  0.9;
    p.v2 = -0.1;
    p.a1 = 1./3.;
    p.a2 = 2./3.;
    myparas.push_back(p);
  }
  
  {
    parameters p;
    p.x  =   -1E-6;
    p.l  = 1000E-6;
    p.v1 = -100;
    p.v2 = -100;
    p.a1 = 1./3.;
    p.a2 = 2./3.;
    myparas.push_back(p);
  }
}

















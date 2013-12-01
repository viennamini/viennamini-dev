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


#include "viennamini/simulator.hpp"
#include "viennamini/utils/timer.hpp"

#include "templates/capacitor2d.hpp"

#include <sstream>

struct xlv
{
  xlv(double x, double y, double l, double h, double c, double v1, double v2) :
    x_(x), y_(y), l_(l), h_(h), c_(c), v1_(v1), v2_(v2) {}

  double x_;
  double y_;
  double l_;
  double h_;
  double c_;
  double v1_;
  double v2_;
};

struct a14
{
  a14(double a1, double a2, double a3, double a4) : 
    a1_(a1), a2_(a2), a3_(a3), a4_(a4) {}
  double a1_;
  double a2_;
  double a3_;
  double a4_;
};

struct b12
{
  b12(double b1, double b2) : 
    b1_(b1), b2_(b2) {}
  double b1_;
  double b2_;
};

struct epsr
{
  epsr(double epsr1, double epsr2, double epsr3) : 
    epsr1_(epsr1), epsr2_(epsr2), epsr3_(epsr3) {}
  double epsr1_;
  double epsr2_;
  double epsr3_;
};

struct test_driver
{
  void register_xlv(xlv const& new_xlv)
  {
    xlvs_.push_back(new_xlv);
  }
  
  void register_a14(a14 const& new_a14)
  {
    a14s_.push_back(new_a14);
  }
  
  void register_b12(b12 const& new_b12)
  {
    b12s_.push_back(new_b12);
  }
  
  void register_epsr(epsr const& new_epsr)
  {
    epsrs_.push_back(new_epsr);
  }
  
  void run(bool write_vtk_files = true)
  {
    int cnt = 1;
    for(std::vector<xlv>::iterator xlviter = xlvs_.begin(); 
        xlviter != xlvs_.end(); xlviter++)
    {
      for(std::vector<a14>::iterator a14iter = a14s_.begin();
          a14iter != a14s_.end(); a14iter++)
      {
        for(std::vector<b12>::iterator b12iter = b12s_.begin();
            b12iter != b12s_.end(); b12iter++)
        {
          for(std::vector<epsr>::iterator epsriter = epsrs_.begin();
              epsriter != epsrs_.end(); epsriter++)
          {
            std::cout << "-- Executing test: " << cnt << " of " << this->size() << std::endl;
            this->test(*xlviter, *a14iter, *b12iter, *epsriter, write_vtk_files, cnt);
            cnt++;
          }
        }
      }
    }
  }

  void test(xlv& myxlv, a14& mya14, b12& myb12, epsr& myepsr, bool write_vtk_files, int cnt)
  {
    viennamini::simulator  mysim(new viennamini::capacitor2d());
    mysim.device().read_material_library("../../examples/materials.xml");

    typedef viennamini::device_template::point_type PointType;
    mysim.device_generator().geometry_properties()["P1"]   = PointType(myxlv.x_                    , myxlv.y_         );
    mysim.device_generator().geometry_properties()["P2"]   = PointType(myxlv.x_+myxlv.l_           , myxlv.y_         );
    mysim.device_generator().geometry_properties()["P3"]   = PointType(myxlv.x_+myxlv.l_           , myxlv.y_+myxlv.h_);
    mysim.device_generator().geometry_properties()["P4"]   = PointType(myxlv.x_                    , myxlv.y_+myxlv.h_);
    mysim.device_generator().geometry_properties()["PI1"]  = PointType(myxlv.x_+myxlv.l_*mya14.a1_ , myxlv.y_); 
    mysim.device_generator().geometry_properties()["PI2"]  = PointType(myxlv.x_+myxlv.l_*mya14.a2_ , myxlv.y_);
    mysim.device_generator().geometry_properties()["PI3"]  = PointType(myxlv.x_+myxlv.l_*mya14.a3_ , myxlv.y_+myxlv.h_);
    mysim.device_generator().geometry_properties()["PI4"]  = PointType(myxlv.x_+myxlv.l_*mya14.a4_ , myxlv.y_+myxlv.h_);
    mysim.device_generator().geometry_properties()["PC1"]  = PointType(myxlv.x_                    , myxlv.y_+myxlv.h_*myb12.b1_);
    mysim.device_generator().geometry_properties()["PC11"] = PointType(myxlv.x_-myxlv.c_           , myxlv.y_+myxlv.h_*myb12.b1_);
    mysim.device_generator().geometry_properties()["PC12"] = PointType(myxlv.x_-myxlv.c_           , myxlv.y_);
    mysim.device_generator().geometry_properties()["PC2"]  = PointType(myxlv.x_+myxlv.l_           , myxlv.y_+myxlv.h_*myb12.b2_);
    mysim.device_generator().geometry_properties()["PC21"] = PointType(myxlv.x_+myxlv.l_+myxlv.c_  , myxlv.y_+myxlv.h_*myb12.b2_);
    mysim.device_generator().geometry_properties()["PC22"] = PointType(myxlv.x_+myxlv.l_+myxlv.c_  , myxlv.y_+myxlv.h_);

    mysim.device().set_permittivity(2, myepsr.epsr1_);
    mysim.device().set_permittivity(3, myepsr.epsr2_);
    mysim.device().set_permittivity(4, myepsr.epsr3_);

    // generate the device, i.e., mesh the geometry and assign segment roles
    //
    mysim.device_generator().generate();
    
    // set contact potentials
    //
    mysim.contact_potential(1) = myxlv.v1_;
    mysim.contact_potential(5) = myxlv.v2_;

    if(write_vtk_files)
    {
      std::stringstream sstr;
      sstr << cnt;
      mysim.set_output_filename_prefix("capacitor1d_test_"+sstr.str());
    }
    
    // perform the simulation
    //
    mysim.run();
  }
  
  int size()
  {
    return xlvs_.size() * a14s_.size() * b12s_.size() * epsrs_.size();
  }
  
  std::vector<xlv>    xlvs_;
  std::vector<a14>    a14s_;
  std::vector<b12>    b12s_;
  std::vector<epsr>   epsrs_;
};

int main()
{
  test_driver mydrv;
  
  mydrv.register_xlv(xlv(-0.1E-6, -0.1E-6, 10E-6, 100E-6, 5E-6, -0.1, -0.1));
  mydrv.register_xlv(xlv(-0.1E-6, -0.1E-6, 10E-6, 100E-6, 5E-6, -0.1,  0.4));
  mydrv.register_xlv(xlv(-0.1E-6, -0.1E-6, 10E-6, 100E-6, 5E-6, -0.1,  0.9));
  mydrv.register_xlv(xlv(  -1E-6,  -10E-6, 1000E-6, 100E-6, 5E-6, -100, -100));
  mydrv.register_xlv(xlv(  -1E-6,  -10E-6, 1000E-6, 100E-6, 5E-6, -100,  400));
  mydrv.register_xlv(xlv(  -1E-6,  -10E-6, 1000E-6, 100E-6, 5E-6, -100,  900));

  mydrv.register_a14(a14(1./3., 2./3., 5./6., 1./6.));
  mydrv.register_a14(a14(9./100., 1./10., 90./100., 1./2.));
  
  mydrv.register_b12(b12(1./10., 1./10.));
  mydrv.register_b12(b12(9./10., 9./10.));

  mydrv.register_epsr(epsr(3.9, 11.7, 3.9));
  mydrv.register_epsr(epsr(35.1, 11.7, 3.9));

  viennamini::timer timer;
  timer.start();
  mydrv.run(true);
  std::cout << "Capacitor 2D tests: Finished executing " << mydrv.size() << " tests in " << timer.get() << " s" << std::endl;
}



















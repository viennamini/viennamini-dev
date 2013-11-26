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

#include "templates/capacitor1d.hpp"

#include <sstream>

struct xlv
{
  xlv(double x, double l, double c, double v1, double v2) :
    x_(x), l_(l), c_(c), v1_(v1), v2_(v2) {}

  double x_;
  double l_;
  double c_;
  double v1_;
  double v2_;
};

struct a1a2
{
  a1a2(double a1, double a2) : a1_(a1), a2_(a2) {}
  double a1_;
  double a2_;
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
  void register_xlvs(double x, double l, double c, double v1, double v2)
  {
    xlvs_.push_back(xlv(x, l, c, v1, v2));
  }
  
  void register_a1a2(double a1, double a2)
  {
    a1a2s_.push_back(a1a2(a1, a2));
  }
  
  void register_epsr(double epsr1, double epsr2, double epsr3)
  {
    epsrs_.push_back(epsr(epsr1, epsr2, epsr3));
  }
  
  void run(bool write_vtk_files)
  {
    int cnt = 1;
    for(std::vector<xlv>::iterator xlviter = xlvs_.begin(); 
        xlviter != xlvs_.end(); xlviter++)
    {
      for(std::vector<a1a2>::iterator a1a2iter = a1a2s_.begin();
          a1a2iter != a1a2s_.end(); a1a2iter++)
      {
        for(std::vector<epsr>::iterator epsriter = epsrs_.begin();
            epsriter != epsrs_.end(); epsriter++)
        {
          std::cout << "-- Executing test: " << cnt << " of " << this->size() << std::endl;
          this->test(*xlviter, *a1a2iter, *epsriter, write_vtk_files, cnt);
          cnt++;
        }
      }
    }
  }

  void test(xlv& myxlv, a1a2& mya1a2, epsr& myepsr, bool write_vtk_files, int cnt)
  {
    std::string material_library_file = "../../examples/materials.xml";
    viennamini::device_template_handle device_generator(new viennamini::capacitor1d(material_library_file));

    typedef viennamini::device_template::point_type PointType;
    device_generator->geometry_properties()["C11"]  = PointType(myxlv.x_-myxlv.c_);
    device_generator->geometry_properties()["C1"]   = PointType(myxlv.x_);
    device_generator->geometry_properties()["I1"]   = PointType(myxlv.x_+myxlv.l_*mya1a2.a1_);
    device_generator->geometry_properties()["I2"]   = PointType(myxlv.x_+myxlv.l_*mya1a2.a2_);
    device_generator->geometry_properties()["C2"]   = PointType(myxlv.x_+myxlv.l_);
    device_generator->geometry_properties()["C21"]  = PointType(myxlv.x_+myxlv.l_+myxlv.c_);

    device_generator->generate();

    viennamini::config_handle & myconfig = device_generator->config();
    viennamini::device_handle & mydevice = device_generator->device();

    mydevice->set_contact_potential(1, myxlv.v1_);
    mydevice->set_contact_potential(5, myxlv.v2_);

    mydevice->set_permittivity(2, myepsr.epsr1_);
    mydevice->set_permittivity(3, myepsr.epsr2_);
    mydevice->set_permittivity(4, myepsr.epsr3_);

    viennamini::simulator   mysim;

    mysim.set_device(mydevice);
    mysim.set_config(myconfig);

    mysim.run();

    if(write_vtk_files)
    {
      std::stringstream sstr;
      sstr << cnt;
      mysim.set_output_filename_prefix("capacitor1d_test_"+sstr.str());
    }
  }
  
  int size()
  {
    return xlvs_.size() * a1a2s_.size() * epsrs_.size();
  }
  
  std::vector<xlv>    xlvs_;
  std::vector<a1a2>   a1a2s_;
  std::vector<epsr>   epsrs_;
};

int main()
{
  test_driver mydrv;
  
  mydrv.register_xlvs(-0.1E-6, 10E-6, 1E-6, -0.1, -0.1);
  mydrv.register_xlvs(-0.1E-6, 10E-6, 1E-6, -0.1,  0.4);
  mydrv.register_xlvs(-0.1E-6, 10E-6, 1E-6, -0.1,  0.9);
  mydrv.register_xlvs(-1E-6, 1000E-6, 100E-6, -100, -100);
  mydrv.register_xlvs(-1E-6, 1000E-6, 100E-6, -100,  400);
  mydrv.register_xlvs(-1E-6, 1000E-6, 100E-6, -100,  900);

  mydrv.register_a1a2(1./3.,   2./3.);
  mydrv.register_a1a2(9./100., 1./10.);

  mydrv.register_epsr(3.9, 11.7, 3.9);
  mydrv.register_epsr(35.1, 11.7, 3.9);

  viennamini::timer timer;
  timer.start();
  mydrv.run(true);
  std::cout << "Capacitor 1D tests: Finished executing " << mydrv.size() << " tests in " << timer.get() << " s" << std::endl;
}


















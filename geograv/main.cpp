// Example of using the GeographicLib::GravityModel class
// This requires that the egm96 gravity model be installed; see
// https://geographiclib.sourceforge.io/html/gravity.html#gravityinst
#include <iostream>
#include <exception>
#include <GeographicLib/GravityModel.hpp>
#include "include/geograv.hpp"
#include "include/EGM96.hpp"
using namespace std;
using namespace GeographicLib;






const geograv::Coeff<400> EGM96_trunc(EGM96);

int main() {
  try {
    GravityModel grav("egm96");
    double x= 5.2124E6L;
    double y= 4.0340E6L;
    double z= -1.9663E6L;
    double gx, gy, gz;
    double pot=grav.V(x,y,z, gx, gy, gz);
    printf("gx: %.15e\n",gx);
    printf("gy: %.15e\n",gy);
    printf("gz: %.15e\n",gz);
    printf("pot: %.15e\n",pot);
    geograv::Vector r_test;
    r_test.x=x;
    r_test.y=y;
    r_test.z=z;
    geograv::Vector g_test;
    double pot_test=geograv::GeoGrav(r_test, g_test,EGM96_trunc,true);
    // g_test.x+=-6.385064555426945e+00;
    // g_test.y+=-4.941552915469322e+00;
    // g_test.z+=2.408670177909601e+00;
    // g_test.x+=-6.390337372231069e+00;
    // g_test.y+=-4.945633673467142e+00;
    // g_test.z+=2.417386322380815e+00;
    printf("gx_test: %.15e\n",g_test.x);
    printf("gy_test: %.15e\n",g_test.y);
    printf("gz_test: %.15e\n",g_test.z);
    printf("pot_test: %.15e\n",pot_test);
    printf("gx_dif: %.15e\n",g_test.x-gx);
    printf("gy_dif: %.15e\n",g_test.y-gy);
    printf("gz_dif: %.15e\n",g_test.z-gz);
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}

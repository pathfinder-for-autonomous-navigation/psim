#include "geograv.hpp"
#include "EGM96.hpp"


void setup() {
    // put your setup code here, to run once:
    pinMode(1,INPUT);
    Serial.begin(9600);
}

void loop() {
    int val= digitalRead(1);
    double x= 5.2124E6L;
    double y= 4.0340E6L;
    double z= -1.9663E6L;
    geograv::Vector r_test;
    r_test.x=x+val*1.0E-9L;
    r_test.y=y;
    r_test.z=z;
    geograv::Vector g_test;
    int starttime=micros();
    int starttimemil=millis();
    static constexpr geograv::Coeff<20> EGM96_truncated(EGM96);
    double pot_test=geograv::GeoGrav(r_test, g_test,EGM96_truncated,true);
    int endtime=micros();
    int endtimemil=millis();
    Serial.printf("gx_test: %.15e\n",g_test.x+6.390258966905924e+00L);
    Serial.printf("gy_test: %.15e\n",g_test.y+4.945665083353001e+00L);
    Serial.printf("gz_test: %.15e\n",g_test.z-2.417276137588813e+00L);
    Serial.printf("pot_test: %.15e\n",pot_test);
    Serial.print("time in micro seconds: ");
    Serial.println(endtime-starttime);
    Serial.print("time in milli seconds: ");
    Serial.println(endtimemil-starttimemil);
    delay(2000);

}

/*
MIT License

Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * \file Orbit.h
 * \author Nathan Zimmerberg
 * \date 10 APR 2020
 * \brief A class to handle most of the logic related to storing, serializing, propagating, and checking validity of an orbit.
 */

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <array>
#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include "JacobianHelpers/jacobian_autocoded.h"

#include "geograv.hpp"
#include "GGM05S.hpp"

namespace orb
{

GNC_TRACKED_CONSTANT(constexpr static int, PANGRAVORDER,40);
GNC_TRACKED_CONSTANT(constexpr geograv::Coeff<PANGRAVORDER>, PANGRAVITYMODEL, static_cast<geograv::Coeff<PANGRAVORDER>>(GGM05S));

GNC_TRACKED_CONSTANT(const double, MAXORBITRADIUS, 6378.0E3L+1000.0E3);

GNC_TRACKED_CONSTANT(const double, MINORBITRADIUS, 6378.0E3L);


/**
 * Class to handle most of the logic related to storing, serializing, propagating, and checking validity of an orbit.
 */
class Orbit {
  public:
    /** Position of the sat (m).*/
    lin::Vector3d _recef= lin::nans<lin::Vector3d>();

    /** Velocity of the sat (m/s).*/
    lin::Vector3d _vecef= lin::nans<lin::Vector3d>();

    /** Time since gps epoch (ns).*/
    uint64_t _ns_gps_time{0};

    /** Validity of the orbit.
     * A valid orbit has finite and real position and velocity, is in low
     * earth orbit, and has a reasonable time stamp (within 20 years of pan epoch).
     * The validity check should not reject 
     * gps readings due to reasonable noise of:
     * TODO add max expected gps error see issue #372
     *
     * Low earth orbit is a Orbit that stays between MINORBITRADIUS and MAXORBITRADIUS.
     */
    bool _valid{false};

    /** Returns time since gps epoch (ns).
     * The Orbit must be not propagating.
     * 
     * grav calls: 0 */
    uint64_t nsgpstime() const{
        return _ns_gps_time;
    }

    /** Return position of the sat (m).
     * The Orbit must be not propagating.
     * 
     * grav calls: 0 */
    lin::Vector3d recef() const{
        return _recef;
    }

    /** Return velocity of the sat (m/s).
     * The Orbit must be not propagating.
     * 
     * grav calls: 0 */
    lin::Vector3d vecef() const{
        return _vecef;
    }

    /** Return true if the Orbit is valid.
     * A valid orbit has finite and real position and velocity, is in low
     * earth orbit, and has a reasonable time stamp (within 20 years of pan epoch).
     * The validity check should not reject 
     * gps readings due to reasonable noise of:
     * TODO add max expected gps error see issue #372
     *
     * Low earth orbit is a Orbit that stays between MINORBITRADIUS and MAXORBITRADIUS.
     * 
     * grav calls: 0 */
    bool valid() const{
        return _valid;
    }
    
    /** 
     * Helper function calculate if the Orbit is valid, see valid().
     * If the orbit is invalid set the orbit to the default values.
     * 
     * grav calls: 0 */
    void _check_validity(){
        double r2= lin::fro(_recef);
        //TODO add checks for time and velocity see issue #372
        //note if position is NAN, these checks will be false.
        if (r2<MAXORBITRADIUS*MAXORBITRADIUS && r2>MINORBITRADIUS*MINORBITRADIUS){
            _valid= true;
        } else {
            _valid= false;
            _recef= lin::nans<lin::Vector3d>();
            _vecef= lin::nans<lin::Vector3d>();
            _ns_gps_time= 0;
        }
    }

    /** Gravity function in International Terrestrial Reference System coordinates.
     * 
     * grav calls: 1
     * @param[in] r_ecef (Above the surface of earth): The location where the gravity is calculated, units m.
     * @param[out] g_ecef: Acceleration due to gravity, units m/s^2. 
     * @param[out] potential: Gravity potential, the acceleration is the gradient of this (J/kg).
     */
    static void calc_geograv(const lin::Vector3d& r_ecef, lin::Vector3d& g_ecef, double& potential) {
        geograv::Vector in;
        geograv::Vector g;
        in.x= r_ecef(0);
        in.y= r_ecef(1);
        in.z= r_ecef(2);
        potential=geograv::GeoGrav(in, g,PANGRAVITYMODEL,true);
        g_ecef(0)= g.x;
        g_ecef(1)= g.y;
        g_ecef(2)= g.z;
    }
    
    /**
     * Construct invalid Orbit.
     * 
     * grav calls: 0
     */
    Orbit(){}

    /**
     * Construct Orbit from time, position, and velocity.
     * Orbit self may be invalid if the inputs are bad, see valid().
     * 
     * grav calls: 0
     * @param[in] ns_gps_time: time since gps epoch (ns).
     * @param[in] r_ecef: position of the center of mass of the sat (m).
     * @param[in] v_ecef: velocity of the sat (m/s).
     */
    Orbit(const uint64_t& ns_gps_time,const lin::Vector3d& r_ecef,const lin::Vector3d& v_ecef): 
        _recef(r_ecef),
        _vecef(v_ecef),
        _ns_gps_time(ns_gps_time) {
        _check_validity();
    }

    /**
     * Apply a deltav to the Orbit.
     * The Orbit must be not propagating.
     * 
     * grav calls: 0
     * @param[in] deltav_ecef: The change in velocity in ecef frame (m/s). 
     */
    void applydeltav(const lin::Vector3d& deltav_ecef){
        _vecef= _vecef+deltav_ecef;
        _check_validity();
    }

    /**
     * Return the specific energy of the Orbit (J/kg).
     * The Orbit must be not propagating.
     * If the Orbit is invalid this returns NAN
     * 
     * grav calls: 1 if Orbit valid, 0 if Orbit invalid
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame (rad/s). 
     */
    double specificenergy(const lin::Vector3d& earth_rate_ecef) const{
        if (valid()){
            double potential;
            lin::Vector3d junk;
            calc_geograv(recef(),junk,potential);
            lin::Vector3d v_ecef0= lin::cross(earth_rate_ecef,recef())+vecef();
            return 0.5L*lin::fro(v_ecef0) - potential;
        }
        else{
            return gnc::constant::nan;
        }
    }

    /************* PROPAGATION ******************/

    /** Gets the dcm to rotate from initial ecef to ecef dt seconds latter.
     * Use equation 2.110 from Markley, Crassidis, Fundamentals of Spacecraft Attitude Determination and Control.
     * 
     * grav calls: 0
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame (rad/s). 
     * @param[in] dt: Change in time (s). 
     * @param[out] A_EI: DCM to rotate from initial ecef to ecef dt seconds latter.
     */ 
    static void relative_earth_dcm_helper(const lin::Vector3d& earth_rate_ecef, const double& dt, lin::Matrix<double, 3, 3>&  A_EI){
        double earth_omega= lin::norm(earth_rate_ecef);
        lin::Vector3d e= earth_rate_ecef/earth_omega;
        double theta= earth_omega*dt;
        double c= std::cos(theta);
        double s= std::sin(theta);
        double e1= e(0);
        double e2= e(1);
        double e3= e(2);
        //equation 2.110 from the Markley Crassidis ADCS book.
        A_EI(0,0)= c+(1-c)*e1*e1;
        A_EI(0,1)= (1-c)*e1*e2+s*e3;
        A_EI(0,2)= (1-c)*e1*e3-s*e2;
        A_EI(1,0)= (1-c)*e2*e1-s*e3;
        A_EI(1,1)= c+(1-c)*e2*e2;
        A_EI(1,2)= (1-c)*e2*e3+s*e1;
        A_EI(2,0)= (1-c)*e3*e1+s*e2;
        A_EI(2,1)= (1-c)*e3*e2-s*e1;
        A_EI(2,2)= c+(1-c)*e3*e3;
    }

    /** Get the jacobian of a shortupdate.
     * This is partially auto code from sympy in JacobianHelpers/jacobian_autocoder.py
     * 
     * The jacobian is calculated assuming point mass earth and constant earth rate of:
     *  w= 1.0e-04L * 0.729211585530000L rad/s in ecef z direction.
     * 
     * grav calls: 0
     * @param[in] r_half: The sats position in ecef0 at the half step (m).
     * @param[in] dt: Time step used (s). 
     * @param[out] jac: 
     * The jacobian of y=f(x) where x and y are vectors
           [r_ecef;
            v_ecef;] in m and m/s.
       and f is the Orbit::shortupdate() function.
     */
    static void _jacobian_helper(const lin::Vector3d& r_half, const  double& dt, lin::Matrix<double, 6, 6>& jac){
        double mu= PANGRAVITYMODEL.earth_gravity_constant;
        double x_h= r_half(0);
        double y_h= r_half(1);
        double z_h= r_half(2);
        double w= 1.0e-04L * 0.729211585530000L; //earths angular rate in z
        jacobian_autocoded(x_h,y_h,z_h,w,mu,dt,jac);
    }

    /**
     * Helper to do a short update of the orbit.
     * The orbit propigator is designed for nearly circular low earth orbit, and
     * ignores all forces except constant gravity from earth.
     * The Orbit must be not propagating and valid.
     * 
     * grav calls: 1
     * @param[in] dt (in the range [-0.2,0.2]): Time step (s).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame (rad/s). 
     * @param[out] r_half_ecef0: The sats position in ecef0 at the half step (m).
     * @param[out] specificenergy: Specific energy of the Orbit at the half step (J/kg).
     */
    void _shortupdate_helper(const double& dt,const lin::Vector3d& earth_rate_ecef, lin::Vector3d& r_half_ecef0, double& specificenergy){
        double mu= PANGRAVITYMODEL.earth_gravity_constant;
        // step 1a ecef->ecef0
        lin::Vector3d r_ecef0= _recef;
        lin::Vector3d v_ecef0= lin::cross(earth_rate_ecef,_recef)+_vecef;
        // step 1b get circular orbit reference
        double energy= 0.5*lin::dot(v_ecef0,v_ecef0)-mu/lin::norm(r_ecef0);
        double a= -mu/2/energy;
        lin::Vector3d h_ecef0= lin::cross(r_ecef0,v_ecef0);
        lin::Vector3d x= r_ecef0;
        x= x/lin::norm(x)*a;
        lin::Vector3d y= lin::cross(h_ecef0,r_ecef0);
        y= y/lin::norm(y)*a;
        lin::Vector3d omega= h_ecef0/lin::norm(h_ecef0)*std::sqrt(mu/(a*a*a));
        lin::Vector3d rel_r= r_ecef0-x;
        lin::Vector3d rel_v= v_ecef0-lin::cross(omega,x);
        // step 2 drift
        rel_r= rel_r+rel_v*dt*0.5;
        // step 3a calc acceleration at the half step
        double t= 0.5*dt;
        double theta= t*lin::norm(omega);
        double costheta= std::cos(theta);
        double sintheta= std::sin(theta);
        lin::Vector3d orb_r= x*costheta+y*sintheta;
        r_ecef0= rel_r+orb_r;
        r_half_ecef0= r_ecef0;
        lin::Matrix<double, 3, 3> dcm_ecefhalf_ecef0;
        relative_earth_dcm_helper(earth_rate_ecef, t, dcm_ecefhalf_ecef0);
        lin::Vector3d pos_ecef= dcm_ecefhalf_ecef0*r_ecef0;
        lin::Vector3d g_ecef;
        double potential;
        calc_geograv(pos_ecef, g_ecef, potential);
        //convert to ECEF0
        lin::Vector3d g_ecef0= lin::transpose(dcm_ecefhalf_ecef0)*g_ecef + mu*orb_r/(a*a*a);
        // step 3b kick velocity
        lin::Vector3d relhalf_v= rel_v + g_ecef0*dt*0.5;
        double halfstepke= lin::fro(relhalf_v+lin::cross(omega,orb_r))*0.5;
        rel_v= rel_v + g_ecef0*dt;
        // step 4 drift
        rel_r= rel_r+rel_v*dt*0.5;
        // step 5a get back absolute orbit
        double cos2theta= costheta*costheta-sintheta*sintheta;
        double sin2theta= 2*sintheta*costheta;
        orb_r= x*cos2theta+y*sin2theta;
        r_ecef0= rel_r+ orb_r;
        v_ecef0= rel_v+ lin::cross(omega,orb_r);
        // step 5b rotate back to ecef
        _recef= dcm_ecefhalf_ecef0*dcm_ecefhalf_ecef0*r_ecef0;
        _vecef= dcm_ecefhalf_ecef0*dcm_ecefhalf_ecef0*v_ecef0;
        // step 6 remove cross r term from velocity
        _vecef= _vecef-lin::cross(earth_rate_ecef,_recef);
        specificenergy= halfstepke - potential;
    }

    /**
     * Do a short update of the orbit.
     * The orbit propigator is designed for nearly circular low earth orbit, and
     * ignores all forces except constant gravity from earth.
     * The jacobians are calculated assuming point mass earth and constant earth rate of:
     *  w= 1.0e-04L * 0.729211585530000L rad/s in ecef z direction.
     * The Orbit must be not propagating.
     * 
     * grav calls: 1 if Orbit valid, 0 if Orbit invalid
     * @param[in] dt_ns (in the range [-2E8,2E8]): Time step (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame (rad/s). 
     * @param[out] jac: 
     * The jacobian of y=f(x) where x and y are vectors
           [r_ecef;
            v_ecef;] in m and m/s.
       and f is the shortupdate function.
     * @param[out] specificenergy: Specific energy of the Orbit at the half step (J/kg).
     */
    void shortupdate(int32_t dt_ns,const lin::Vector3d& earth_rate_ecef, double& specificenergy, lin::Matrix<double, 6, 6>& jac){
        if (!valid()){
            jac= lin::nans<lin::Matrix<double, 6, 6>>();
            specificenergy= gnc::constant::nan;
            return;
        }
        assert(dt_ns<=200'000'000 && dt_ns>=-200'000'000);
        _ns_gps_time+= dt_ns;
        lin::Vector3d r_half_ecef0;
        double dt= double(dt_ns)*1E-9L;
        //_jacobian_helper(_recef,_vecef ,dt, jac); 
        _shortupdate_helper(dt,earth_rate_ecef, r_half_ecef0, specificenergy);
        _jacobian_helper(r_half_ecef0 ,dt, jac); 
    }

    /**
     * Do a short update of the orbit.
     * The orbit propigator is designed for nearly circular low earth orbit, and
     * ignores all forces except constant gravity from earth.
     * The Orbit must be not propagating.
     * 
     * grav calls: 1 if Orbit valid, 0 if Orbit invalid
     * @param[in] dt_ns (in the range [-2E8,2E8]): Time step (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame (rad/s). 
     * @param[out] specificenergy: Specific energy of the Orbit at the half step (J/kg).
     */
    void shortupdate(int32_t dt_ns,const lin::Vector3d& earth_rate_ecef, double& specificenergy){
        if (!valid()){
            specificenergy= gnc::constant::nan;
            return;
        }
        assert(dt_ns<=200'000'000 && dt_ns>=-200'000'000);
        _ns_gps_time+= dt_ns;
        lin::Vector3d r_half_ecef0;
        double dt= double(dt_ns)*1E-9L;
        _shortupdate_helper(dt,earth_rate_ecef, r_half_ecef0, specificenergy);
    }

    //some varables to handle long updates
    int _longstep{0};
    int _numgravcallsleft{0};
    uint64_t _totalduration{0};
    uint64_t _targetgpstime{0};
    double _currentdt{0};
    double _a;
    lin::Vector3d _x;
    lin::Vector3d _y;
    lin::Vector3d _omega;
    lin::Vector3d _earth_rate_ecef=lin::nans<lin::Vector3d>();
    GNC_TRACKED_CONSTANT(static const int64_t,maxlongtimestep,105'000'000'000LL);
    GNC_TRACKED_CONSTANT(static const int64_t,maxshorttimestep,200'000'000LL);


    /**
     * Puts the Orbit in propagating mode.
     * call finishpropagating() or onegravcall() numgravcallsleft() times to finish the propagating.
     * This function can also be used to change the end time of a propagating Orbit.
     * This doesn't do anything if the orbit is invalid.
     * 
     * grav calls: 0
     * @param[in] end_gps_time_ns: Time to propagate to (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame ignored if already propagating(rad/s).
     */
    void startpropagating(const uint64_t& end_gps_time_ns, const lin::Vector3d& earth_rate_ecef){
        if (!valid()){
            return;
        }
        _targetgpstime= end_gps_time_ns;
        //now get _numgravcallsleft
        int64_t deltatime= _targetgpstime-_ns_gps_time;
        int64_t absdeltatime= std::abs(deltatime);
        if((_numgravcallsleft==0) && (deltatime!=0)){
            //starting convert ecef to ecef0
            _earth_rate_ecef= earth_rate_ecef;
            _vecef= lin::cross(earth_rate_ecef,_recef)+_vecef;
        }
        _numgravcallsleft= _longstep?(7-_longstep):0;
        if (absdeltatime>maxshorttimestep*6){
            //how many long steps
            int n= absdeltatime/maxlongtimestep;//full long steps
            _numgravcallsleft+=n*7;
            absdeltatime-=n*maxlongtimestep;
        }

        if (absdeltatime==0){
            return;
        }
        _numgravcallsleft+=std::min((absdeltatime-1)/maxshorttimestep+1,7LL);
    }

    /**
     * Return the number of grav calls needed to finish propagating.
     * If not propagating Return 0.
     * 
     * grav calls: 0
     */
    int numgravcallsleft(){
        return _numgravcallsleft;
    }

    /**
     * If propagating call the gravity model once
     * to move the propagtor forward, otherwise do nothing.
     * High order integrators from:
     * https://doi.org/10.1016/0375-9601(90)90092-3
     * 
     * grav calls: 1 if propagating, 0 if not propagating
     */
    void onegravcall(){
        double mu= PANGRAVITYMODEL.earth_gravity_constant;
        //high order integrators
        //https://doi.org/10.1016/0375-9601(90)90092-3
        static const std::array<double,7> d{{0.784513610477560L,
                                        0.235573213359357L,
                                        -1.177679984178870L,
                                        1.315186320683906L,
                                        -1.177679984178870L,
                                        0.235573213359357L,
                                        0.784513610477560L}};
        if (!valid()){
            return;
        }
        if (_numgravcallsleft==0){
            //done propagating
            return;
        }
        //now what dt should be used?
        double dt;
        //aready in ecef0
        lin::Vector3d v_ecef0=_vecef;
        lin::Vector3d r_ecef0=_recef;
        if (_longstep==0){
            //not inside a long step
            //get reference orbit
            double energy= 0.5*lin::dot(v_ecef0,v_ecef0)-mu/lin::norm(r_ecef0);
            _a= -mu/2/energy;
            lin::Vector3d h_ecef0= lin::cross(r_ecef0,v_ecef0);
            _x= r_ecef0;
            _x= _x/lin::norm(_x)*a;
            _y= lin::cross(h_ecef0,r_ecef0);
            _y= _y/lin::norm(_y)*a;
            _omega= h_ecef0/lin::norm(h_ecef0)*std::sqrt(mu/(a*a*a));
            //store relative positions and velocities
            _recef= r_ecef0-_x;
            _vecef= v_ecef0-lin::cross(_omega,_x);
            int64_t deltatime= _targetgpstime-_ns_gps_time;
            int signofdt= (deltatime<0)?-1:1;
            int64_t currentdtns;
            if (_numgravcallsleft>=7){
                // do a long step
                if (std::abs(deltatime)>=std::abs(maxlongtimestep)){
                    // take a full time step
                    currentdtns= signofdt*maxlongtimestep;
                }else{
                    // take a partial time step
                    currentdtns= deltatime;
                }
                _ns_gps_time+=currentdtns;
                _currentdt= double(currentdtns)*1E-9L;
                dt= _currentdt*d[_longstep];
                _longstep++;
            }else{
                // do a short step
                if (std::abs(deltatime)>=std::abs(maxshorttimestep)){
                    // take a full time step
                    currentdtns= signofdt*maxshorttimestep;
                }else{
                    // take a partial time step
                    currentdtns= deltatime;
                }
                _ns_gps_time+=currentdtns;
                dt= double(currentdtns)*1E-9L;
            }
        }else{
            //in the middle of a long step
            dt= _currentdt*d[_longstep];
            _longstep++;
            if (_longstep>=7) _longstep=0;    
        }
        lin::Vector3d junk;
        double alsojunk;
        _shortupdate_helper(dt,_earth_rate_ecef, junk, alsojunk);
        _numgravcallsleft--;
        if(_numgravcallsleft==0){
            //convert back to ecef TODO
        }
        return;
    }

    /**
     * Do all remaining grav calls to finish propagating.
     * 
     * grav calls: numgravcallsleft()
     */
    void finishpropagating(){
        while(numgravcallsleft()){
            onegravcall();
        }
    }
};
}
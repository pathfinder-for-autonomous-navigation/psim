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
 * \file GPSPosVelEstimator.h
 * \author Nathan Zimmerberg
 * \date 27 APR 2020
 * \brief An estimator of the current Orbit from GPS measurements.
 */

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <lin/core.hpp>
#include <lin/generators.hpp>
#include "Orbit.h"
#include <array>
#include <utility>
#include "kalman_utl.h"
#include <gnc/constants.hpp>

namespace orb
{



/**
 * An estimator of the current Orbit from only GPS measurements.
 * 
 * 
 */
class GPSPosVelEstimator {
    public:
    /**
     * Construct GPSPosVelEstimator.
     *
     * grav calls: 0
     */
    GPSPosVelEstimator(){}

    //P= S*S^T
    lin::Matrixd<6,6> S= lin::nans<lin::Matrixd<6,6>>();
    Orbit estorb= Orbit();

    /** Return the best estimate of the Orbit.
     * 
     * grav calls: 0
     */
    Orbit best_estimate() const{
        return estorb;
    }

    /** The max number of grav calls the estimator will use.*/
    GNC_TRACKED_CONSTANT(static const int, MAXGRAVCALLS,5);


    /// \private
    /** 
     * Propagate state and covariance square root
     * If the time step is negative, no process noise is added or removed.
     * grav calls: 1 or 0
     * @param[in] dt_ns (in the range [-2E8,2E8]): Time step (ns).
     */
    void _propagate_helper(int32_t dt_ns,const lin::Vector3d& earth_rate_ecef,const double& processnoise_r,const double& processnoise_v){
        if (dt_ns==0){
            return;
        }
        lin::Matrixd<6,6> jac;
        double junk;
        estorb.shortupdate(dt_ns,earth_rate_ecef,junk,jac);
        S= (jac*S).eval();//P= S*S^T
        if (dt_ns>0) {
            double dt= ((double)dt_ns)*1E-9;
            lin::Matrixd<6,6> Q= lin::zeros<lin::Matrixd<6,6>>();
            sr= processnoise_r*dt;
            sv= processnoise_v*dt;
            Q(0,0)= sr;
            Q(1,1)= sr;
            Q(2,2)= sr;
            Q(3,3)= sv;
            Q(4,4)= sv;
            Q(5,5)= sv;
            matrix_hypot(S,Q,S);
        }
    }

    /// \private 
    /**
     * reset from gps data
     * @param[in] gps_data(valid): gps data
     * @param[in] gps_r_stddev(strictly positive and finite): gps position standard deviation (m).
     * @param[in] gps_v_stddev(strictly positive and finite): gps velocity standard deviation (m/s).
     */
    void _reset_from_gpsdata_helper(const Orbit& gps_data, const double& gps_r_stddev, const double& gps_v_stddev){
        estorb= gpsdata;
        S= lin::zeros<lin::Matrixd<6,6>>();
        S(0,0)= gps_r_stddev;
        S(1,1)= gps_r_stddev;
        S(2,2)= gps_r_stddev;
        S(3,3)= gps_v_stddev;
        S(4,4)= gps_v_stddev;
        S(5,5)= gps_v_stddev;
    }

    /**
     * Input GPS data
     * 
     * grav calls: MAXGRAVCALLS or less
     * @param[in] gps_time_ns: Time to propagate to, 0 if not available (ns).
     * @param[in] recef_gps_data: 
     *      GPS position data, NAN if not availible (m).
     * @param[in] vecef_gps_data: 
     *      GPS velocity data, NAN if not availible (m/s).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame ignored for orbits already propagating(rad/s).
     * @param[in] max_dr_fro_valid(strictly positive and finite): 
     *      Max position squared valid gps data can be from the estimate before reset(m^2).
     * @param[in] max_dv_fro_valid(strictly positive and finite): 
     *      Max velocity squared valid gps data can be from the estimate before reset((m/s)^2).
     */
    void input(const int64_t& gps_time_ns,
                const lin::Vector3d& recef_gps_data, 
                const lin::Vector3d& vecef_gps_data, 
                const lin::Vector3d& earth_rate_ecef, 
                const double& gps_r_stddev, 
                const double& gps_v_stddev, 
                const double& processnoise_r, 
                const double& processnoise_v,
                const double& max_dr_fro_valid,
                const double& max_dv_fro_valid){
        //test gps_time_ns is valid, if not valid return
        int64_t pan_epoch_gps_ns= gnc::constant::init_gps_week_number*(int64_t)gnc::constant::NANOSECONDS_IN_WEEK
        if (std::abs(pan_epoch_gps_ns-gps_time_ns) > 20LL*52LL*(int64_t)gnc::constant::NANOSECONDS_IN_WEEK){
            return;
        }
        int64_t dt_ns= (int64_t)gps_time_ns-(int64_t)estorb.nsgpstime();
        Orbit gpsdata(gps_time_ns,recef_gps_data,vecef_gps_data);
        //first check if we need to initialize
        if (!estorb.valid() || std::abs(dt_ns)>MAXGRAVCALLS*Orbit.maxshorttimestep){
            //the estimate is invalid, can we reinit?
            if (!gpsdata.valid()){
                return;
            }
            //reset state based on GPS reading
            _reset_from_gpsdata_helper(gpsdata,gps_r_stddev,gps_v_stddev);
            //initialization done
            return;
        }
        assert(estorb.valid());
        //propagate the current estimate
        while(dt_ns){
            int32_t dt = std::min(std::max(dt_ns,-Orbit.maxshorttimestep),Orbit.maxshorttimestep);
            _propagate_helper(dt,earth_rate_ecef,processnoise_r,processnoise_v);
            dt_ns= (int64_t)gps_time_ns-(int64_t)estorb.nsgpstime();
        }
        if (gpsdata.valid()){
            bool reset= false;
            // measurement update
            // check if gpsdata is out of linearity bounds
            /** Max distance squared valid gps data can be from the estimate before reset (m^2). */
            GNC_TRACKED_CONSTANT(static const double, max_dr_fro_valid,1E5*1E5);
            /** Max velocity squared valid gps data can be from the estimate before reset((m/s)^2). */
            GNC_TRACKED_CONSTANT(static const double, max_dv_fro_valid,100*100);
            if(lin::fro(gpsdata.recef()-estorb.recef())<=max_dr_fro_valid && lin::fro(gpsdata.vecef()-estorb.vecef())<= max_dv_fro_valid){
                //within linearity bounds do potter update
                lin::Vectord<6> x;
                lin::ref<3, 1> (x, 0, 0)= estorb.recef();
                lin::ref<3, 1> (x, 3, 0)= estorb.vecef();
                double gps_r_invstddev= 1.0/gps_r_stddev;
                double gps_v_invstddev= 1.0/gps_v_stddev;
                potter_measurement_update(x, S, gpsdata.recef()(0), {1.0,0.0,0.0,0.0,0.0,0.0}, gps_r_invstddev);
                potter_measurement_update(x, S, gpsdata.recef()(1), {0.0,1.0,0.0,0.0,0.0,0.0}, gps_r_invstddev);
                potter_measurement_update(x, S, gpsdata.recef()(2), {0.0,0.0,1.0,0.0,0.0,0.0}, gps_r_invstddev);
                potter_measurement_update(x, S, gpsdata.vecef()(0), {0.0,0.0,0.0,1.0,0.0,0.0}, gps_v_invstddev);
                potter_measurement_update(x, S, gpsdata.vecef()(1), {0.0,0.0,0.0,0.0,1.0,0.0}, gps_v_invstddev);
                potter_measurement_update(x, S, gpsdata.vecef()(2), {0.0,0.0,0.0,0.0,0.0,1.0}, gps_v_invstddev);
                //remake orbit
                estorb= Orbit(estorb.nsgpstime(),lin::ref<3, 1> (x, 0, 0),lin::ref<3, 1> (x, 3, 0));
                if(!lin::all(lin::isfinite(S)) || !estorb.valid()){
                    reset=true;
                }
            } else {
                reset=true;
            }
            if(reset){
                _reset_from_gpsdata_helper(gpsdata,gps_r_stddev,gps_v_stddev);
            }  
        }
    }

};
} //namespace orb
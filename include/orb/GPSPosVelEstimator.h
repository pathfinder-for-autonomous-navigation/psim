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
#include <lin/core.hpp>
#include <lin/generators.hpp>
#include "Orbit.h"
#include <array>
#include <utility>

namespace orb
{

/** The max number of control cycles a GPS readings time stamp can be under normal conditions.*/
GNC_TRACKED_CONSTANT(constexpr static int, MAXGPS_CONTROLCYCLE_SLACK,5);

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

    // These buffers are needed in case a GPS reading is from the past
    // Because of slack on the GPS reading
    //P= S*S^T
    std::array<std::pair<Orbit,lin::Matrixd<6,6>>,MAXGPS_CONTROLCYCLE_SLACK> buffer= {std::make_pair(Orbit(),lin::nans<lin::Matrixd<6,6>>())};
    int bufferpointer{0};

    /** Return the best estimate of the Orbit.
     * 
     * grav calls: 0
     */
    Orbit best_estimate() const{
        return buffer[bufferpointer].first;
    }


    /// \private
    /** 
     * Propagate state and covariance square root
     * grav calls: 1 or 0
     * @param[in] dt_ns (in the range [0,2E8]): Time step (ns).
     */
    void _propagate_helper(int32_t dt_ns,const lin::Vector3d& earth_rate_ecef,const double& processnoise_r,const double& processnoise_v){
        lin::Matrixd<6,6> jac;
        double junk;
        double dt= ((double)dt_ns)*1E-9;
        Orbit& x= buffer[bufferpointer].first;
        lin::Matrixd<6,6>& S= buffer[bufferpointer].second;
        if (x.valid()){
            x.shortupdate(dt_ns,earth_rate_ecef,junk,jac);
            S= (jac*S).eval();//P= S*S^T
            lin::Matrixd<6,6>& Q= lin::zeros<lin::Matrixd<6,6>>();
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
     * update state and covariance square root with gpsdata
     * grav calls: 0
     * @param[in] .
     */
    void _sensor_update_helper(const lin::Vector3d& recef_gps_data, const lin::Vector3d& vecef_gps_data, const double& gps_r_invstddiv, const double& gps_v_invstddiv){
        lin::Matrixd<6,6> jac;
        double junk;
        double dt= ((double)dt_ns)*1E-9;
        Orbit& x= buffer[bufferpointer].first;
        lin::Matrixd<6,6>& S= buffer[bufferpointer].second;
        if (x.valid()){
            x.shortupdate(dt_ns,earth_rate_ecef,junk,jac);
            S= (jac*S).eval();//P= S*S^T
            lin::Matrixd<6,6>& Q= lin::zeros<lin::Matrixd<6,6>>();
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




    /**
     * Input GPS data
     * 
     * grav calls: 5 or less
     * @param[in] gps_data: Orbit from GPS
     * @param[in] gps_time_ns: Time to propagate to (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame ignored for orbits already propagating(rad/s).
     */
    void input(const lin::Vector3d& recef_gps_data, 
               const lin::Vector3d& vecef_gps_data, 
               const uint64_t& gps_time_ns, 
               const lin::Vector3d& earth_rate_ecef, 
               const double& gps_r_invstddiv, 
               const double& gps_v_invstddiv, 
               const double& processnoise_r, 
               const double& processnoise_v){
        
        

        

    }




    











}

}
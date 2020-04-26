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
 * \date 26 APR 2020
 * \brief An estimator of the current Orbit from GPS measurements.
 */

#pragma once

#include "Orbit.h"

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

    /**
     * Input the newest Orbit sent from ground.
     * 
     * grav calls: 5 or less
     * @param[in] ground_data: Orbit sent from ground.
     * @param[in] gps_time_ns: Time to propagate to (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame ignored for orbits already propagating(rad/s).
     */
    void input(const Orbit& gps_data, const uint64_t& gps_time_ns, const lin::Vector3d& earth_rate_ecef){ 
        

    }

    /** Return the best estimate of the Orbit.
     * 
     * grav calls: 0
     */
    Orbit best_estimate() const{
    }




    











}

}
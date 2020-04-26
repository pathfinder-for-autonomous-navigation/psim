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
 * \file GroundPropagator.h
 * \author Nathan Zimmerberg
 * \date 23 APR 2020
 * \brief A class to propagate orbits sent from ground.
 */

#pragma once

#include "Orbit.h"

namespace orb
{

/**
 * Class to propagate orbits sent from ground.
 * 
 * The GroundPropagator trys to first minimize the number of grav calls needed to get a 
 * not propagating orbit estimate, and second use the most recently input Orbit.
 * 
 * Implimentation details:
 * 
 * Under normal conditions, this estimator just propagates the most recently uplinked Orbit.
 * in current.
 * 
 * If a new Orbit get uplinked it will normally get put in catching_up, and the 
 * estimator will propagate it to the current time in the background 
 * while still propagating current as the best estimate. 
 * Once catching_up is done propagating it replaces current.
 * 
 * If another Orbit gets uplinked while catching_up is still being propagated 
 * in the background, it gets stored in to_catch_up. This ensures too many Orbits getting 
 * uplinked won't overload the estimator and prevent it from making progress.
 * If to_catch_up takes fewer grav calls to finish propagating than catching_up 
 * it replaces catching_up.
 * Also to_catch_up replaces catching_up if catching_up finishes propagating.
 */
class GroundPropagator {
  public:
    /** Current Orbit estimate.*/
    Orbit current;

    /** Orbit catching up to current time.
     * If catching_up is valid then current is valid.
     * If catching_up is valid then current.numgravcallsleft()<catching_up.numgravcallsleft().
     * If catching_up is valid then catching_up is more recently input than current.
     */
    Orbit catching_up;

    /** Orbit to catch up to current time once catching_up is done catching up.
     * If to_catch_up is valid then catching_up is valid.
     * If to_catch_up is valid then catching_up.numgravcallsleft()<to_catch_up.numgravcallsleft().
     * If to_catch_up is valid then to_catch_up is more recently input than catching_up.
     */
    Orbit to_catch_up;

    /**
     * Construct GroundPropagator.
     *
     * grav calls: 0
     */
    GroundPropagator(){}

    /** Input the newest Orbit sent from ground.
     * 
     * grav calls: 0
     * @param[in] ground_data: Orbit sent from ground.
     * @param[in] gps_time_ns: Time to propagate to (ns).
     * @param[in] earth_rate_ecef: The earth's angular rate in ecef frame ignored for orbits already propagating(rad/s).
     */
    void input(const Orbit& ground_data, const uint64_t& gps_time_ns, const lin::Vector3d& earth_rate_ecef){
        //start propagators
        current.startpropagating(gps_time_ns,earth_rate_ecef);
        catching_up.startpropagating(gps_time_ns,earth_rate_ecef);
        to_catch_up.startpropagating(gps_time_ns,earth_rate_ecef);
        //presort stuff
        if (to_catch_up.valid() && to_catch_up.numgravcallsleft()<=catching_up.numgravcallsleft()){
            catching_up= to_catch_up;
            to_catch_up= Orbit();
        }
        if (catching_up.valid() && catching_up.numgravcallsleft()<=current.numgravcallsleft()){
            current= catching_up;
            catching_up= to_catch_up;
            to_catch_up= Orbit();
        }
        //add to sorted queue
        if (ground_data.valid()){
            to_catch_up= ground_data;
            to_catch_up.startpropagating(gps_time_ns,earth_rate_ecef);
            if(!current.valid() || to_catch_up.numgravcallsleft()<=current.numgravcallsleft()){
                current= to_catch_up;
                to_catch_up= orb::Orbit();
                catching_up= orb::Orbit();
            } else if(!catching_up.valid() || to_catch_up.numgravcallsleft()<=catching_up.numgravcallsleft()){
                catching_up= to_catch_up;
                to_catch_up= orb::Orbit();
            }
        }
    }

    /** Do one grav call to move the propagator towards the current time if needed.
     * The Orbit closest to finish propagating is propagated first.
     * 
     * grav calls: 1 or 0 if total_num_grav_calls_left() == 0 
     */
    void one_grav_call(){
        if(current.numgravcallsleft()){
            current.onegravcall();
        } else { 
            catching_up.onegravcall();
        }
        //sort stuff
        if (catching_up.valid() && catching_up.numgravcallsleft()<=current.numgravcallsleft()){
            current= catching_up;
            catching_up= to_catch_up;
            to_catch_up= Orbit();
        }
    }

    /** Return the total number of grav calls need to finish propagating all the orbits.
     *
     * grav calls: 0
     */
    int total_num_grav_calls_left() const{
        return current.numgravcallsleft()+catching_up.numgravcallsleft()+to_catch_up.numgravcallsleft();
    }

    /** Return the best estimate of the Orbit.
     * 
     * grav calls: 0
     */
    Orbit best_estimate() const{
        return current;
    }

    /** Reset all internal orbits to invalid.
     * 
     * grav calls: 0
     */
    void reset_orbits(){
        Orbit invalid;
        current=invalid;
        catching_up=invalid;
        to_catch_up=invalid;
    }
};
} //namespace orb

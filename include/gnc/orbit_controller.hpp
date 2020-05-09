//
// MIT License
//
// Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file gnc/attitude_estimator.hpp
 *  @author Kyle Krol
 *  Defines the interface for the attitude estimator. */

#ifndef GNC_ORBIT_CONTROLLER_HPP_
#define GNC_ORBIT_CONTROLLER_HPP_

#include <lin/core.hpp>

namespace gnc {

struct OrbitControllerState {
  lin::Vector3d this_r_ecef0, that_r_ecef0, this_r_hat;
  lin::Vector3d this_v_ecef0, that_v_ecef0, this_v_hat;
  lin::Vector3d this_h_ecef0, that_h_ecef0, this_h_hat;
  lin::Matrix3x3d DCM_hill_ecef0;
  /** Defaults everything's value to NaN. */
  OrbitControllerState();
};

struct OrbitControllerData {
  lin::Vector3d r_ecef;
  lin::Vector3d v_ecef;
  lin::Vector3d dr_ecef;
  lin::Vector3d dv_ecef;
  double t;
  /** Defaults everything's value to NaN. */
  OrbitControllerData();
};

struct OrbitActuation {
  lin::Vector3d J_eci;
  float phase_till_next_node;
  /** Defaults everything's value to NaN. */
  OrbitActuation();
};

#ifndef MEX
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, lin::Vector<unsigned int, 4> &on_times);
#else
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, lin::Vector<unsigned int, 4> &on_times,
    double mass, double K_p, double K_d, double K_e, double K_h);
#endif

}  // namespace gnc

#endif

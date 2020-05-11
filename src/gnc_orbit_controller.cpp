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

/** @file gnc_attitude_estimator.cpp
 *  @author Kyle Krol
 *  @author Stewart Aslan
 */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/orbit_controller.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>

#include <cstdint>

namespace gnc {

OrbitControllerState::OrbitControllerState()
: t_last_firing(0),
  this_r_ecef0(lin::nans<decltype(this_r_ecef0)>()),
  that_r_ecef0(lin::nans<decltype(that_r_ecef0)>()),
  this_r_hat(lin::nans<decltype(this_r_hat)>()),
  this_v_ecef0(lin::nans<decltype(this_v_ecef0)>()),
  that_v_ecef0(lin::nans<decltype(that_v_ecef0)>()),
  this_v_hat(lin::nans<decltype(this_v_hat)>()),
  this_h_ecef0(lin::nans<decltype(this_h_ecef0)>()),
  that_h_ecef0(lin::nans<decltype(that_h_ecef0)>()),
  this_h_hat(lin::nans<decltype(this_h_hat)>()),
  DCM_hill_ecef0(lin::nans<decltype(DCM_hill_ecef0)>()) { }

OrbitControllerData::OrbitControllerData()
: t(0),
  r_ecef(lin::nans<decltype(r_ecef)>()),
  v_ecef(lin::nans<decltype(v_ecef)>()),
  dr_ecef(lin::nans<decltype(dr_ecef)>()),
  dv_ecef(lin::nans<decltype(dv_ecef)>()) { }

OrbitActuation::OrbitActuation()
: J_eci(lin::nans<decltype(J_eci)>()),
  phase_till_next_node(gnc::constant::nan_f) { }

#ifndef MEX
static
#endif
void mex_control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation,
    double mass, double K_p, double K_d, double K_e, double K_h) {
  actuation = gnc::OrbitActuation(); // TODO : Implement this
}

#ifndef MEX
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation) {
  mex_control_orbit(state, data, actuation, 0.0, 0.0, 0.0, 0.0, 0.0); // TODO : Implement this
}
#endif

}  // namespace gnc

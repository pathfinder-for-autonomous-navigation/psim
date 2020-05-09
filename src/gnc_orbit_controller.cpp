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
 *  Defines the interface for the attitude estimator. */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <gnc/orbit_controller.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace gnc {

static double energy(lin::Vector3d const &r_eci, lin::Vector3d const &v_eci) {
  return gnc::constant::nan; // TODO : Implement this
}

#ifndef MEX
static
#endif
void control_orbit(struct OrbitControllerState &state, struct OrbitControllerData const &data,
    lin::Vector<unsigned int, 4> &on_times, double mass, double K_p, double K_d,
    double K_e, double K_h) {

  lin::Vector3d &this_r_ecef0 = state.this_r_ecef0, &that_r_ecef0 = state.that_r_ecef0;
  lin::Vector3d &this_r_hat = state.this_r_hat;
  lin::Vector3d &this_v_ecef0 = state.this_v_ecef0, &that_v_ecef0 = state.that_v_ecef0;
  lin::Vector3d &this_v_hat = state.this_v_hat;
  lin::Vector3d &this_h_ecef0 = state.this_h_ecef0, &that_h_ecef0 = state.that_h_ecef0;
  lin::Vector3d &this_h_hat = state.this_h_hat;
  lin::Matrix3x3d &DCM_hill_ecef0 = state.DCM_hill_ecef0;

  // Move to temporary ECEF0
  {
    lin::Vector3f w_earth;
    env::earth_angular_rate(data.t, w_earth);

    this_r_ecef0 = data.r_ecef;
    that_r_ecef0 = data.dr_ecef;
    this_r_hat = this_r_ecef0 / lin::norm(this_r_ecef0);

    this_v_ecef0 = data.v_ecef - lin::cross(w_earth, this_r_ecef0);
    that_v_ecef0 = data.v_ecef + data.dv_ecef - lin::cross(w_earth, this_r_ecef0 + data.dr_ecef);
    this_v_hat = this_v_ecef0 / lin::norm(this_v_ecef0);

    this_h_ecef0 = lin::cross(this_r_ecef0, this_v_ecef0);
    that_h_ecef0 = lin::cross(that_r_ecef0, that_v_ecef0);
    this_h_hat = this_h_ecef0 / lin::norm(this_h_ecef0);
  }

  // Requested delta v in ECEF0
  lin::Vector3d dv_ecef0 = lin::zeros<lin::Vector3d>();

  // Hill frame controller
  {
    lin::Vector3d r_hill, v_hill;
    r_hill = DCM_hill_ecef0 * (that_r_ecef0 - this_r_ecef0);
    v_hill = DCM_hill_ecef0 * (that_v_ecef0 - this_v_ecef0);
    
    dv_ecef0 = dv_ecef0 + ();
  }
  



  lin::Matrix3x3d DCM_hill_ecef0;
  utl::dcm(DCM_hill_ecef0, that_r_ecef0, that_v_ecef0);
  
  lin::Vector3d r_hill = DCM_hill_ecef0 * (that_r_ecef0 - this_r_ecef0);
  lin::Vector3d v_hill = DCM_hill_ecef0 * (that_v_ecef0 - this_v_ecef0);

  lin::Vector3d dv;
  {
    lin::Vector3d this_v_hat  = this_v_ecef0 / lin::norm(this_v_ecef0);

    // TODO : Negative K_d and K_e
    dv = (K_p * r_hill(1) +
          K_d * v_hill(1) +
          K_e * (energy(this_r_ecef0, this_v_ecef0) - energy(that_r_ecef0, that_v_ecef0))
        ) * this_v_hat;

    lin::Vector3d this_r_hat = this_r_ecef0 / lin::norm(this_r_ecef0);
    lin::Vector3d this_h_hat = lin::cross(this_r_hat, this_v_hat);

    lin::Vector3d that_h_proj = lin::cross(that_r_ecef0, that_v_ecef0);
    that_h_proj = that_h_proj - lin::dot(that_h_proj, this_r_hat) * this_r_hat;

    double theta = lin::atan(lin::dot(that_h_proj, lin::cross(this_r_ecef0, this_h)) / );
  }

}

// Spacecraft mass

#ifndef MEX
void control_orbit(struct OrbitControllerState &state, struct OrbitControllerData const &data,
    lin::Vector<unsigned int, 4> &on_times) {

}
#endif

}  // namespace gnc

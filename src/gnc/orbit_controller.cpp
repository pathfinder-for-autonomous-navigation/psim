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
#include <gnc/environment.hpp>
#include <gnc/orbit_controller.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

#include <cstdint>
#include <cmath>
#include <math.h>
#include <iostream>

namespace gnc {

// Initialize constants
static constexpr double mass = 3.7;             // Mass of spacecraft                    (kg)
static constexpr double J_min = 0;              // Minimum impulse                       (N s)
static constexpr double J_max = 2.5e-2;         // Maximum impulse                       (N s)
static constexpr double max_dv = J_max / mass;  // Max velocity change                   (m/s)
static constexpr double min_dv = J_min / mass;  // Min velocity change                   (m/s)
static constexpr double p = 1.0e-6;
static constexpr double d = 5.0e-2;
static constexpr double energy_gain = 5.0e-5;   // Energy gain                           (J)
static constexpr double h_gain = 2.0e-3;        // Angular momentum gain                 (kg m^2/sec)
static constexpr double dt_fire_min = 5 * 60;   // Minimum time between firings          (sec)

// Initial orbital elements
static constexpr double pi = gnc::constant::pi;
static constexpr double e  = 0.001;             // Eccentricity                          (unitless)

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
: J_ecef(lin::nans<decltype(J_ecef)>()),
  phase_till_next_node(gnc::constant::nan_f) { }

/*
 * Calculates orbital energy given position and velocity
 */
static double energy(lin::Vector3d const &r_ecef0, lin::Vector3d const &v_ecef0) {
  // Calculate gravitational potential
  double potential;
  lin::Vector3d acceleration;
  env::gravity(r_ecef0, acceleration, potential);

  // Return energy (E = K + U)
  return 0.5 * lin::dot(v_ecef0, v_ecef0) - potential;
}

#ifndef MEX
static
#endif

/*
 * Orbit Controller
 * Add more info when this is done
 */
void mex_control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation,
    double mass, double K_p, double K_d, double K_e, double K_h) {
    
  lin::Vector3d &this_r_ecef0 = state.this_r_ecef0, &that_r_ecef0 = state.that_r_ecef0;
  lin::Vector3d &this_r_hat = state.this_r_hat;
  lin::Vector3d &this_v_ecef0 = state.this_v_ecef0, &that_v_ecef0 = state.that_v_ecef0;
  lin::Vector3d &this_v_hat = state.this_v_hat;
  lin::Vector3d &this_h_ecef0 = state.this_h_ecef0, &that_h_ecef0 = state.that_h_ecef0;
  lin::Vector3d &this_h_hat = state.this_h_hat;
  lin::Matrix3x3d &DCM_hill_ecef0 = state.DCM_hill_ecef0;

  // Move to temporary ECEF0
  //{
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
  //}
  
  // Calculate the hill frame DCM
  utl::dcm(DCM_hill_ecef0, that_r_ecef0, that_v_ecef0);
 
  // Calculate energy
  double this_energy = energy(this_r_ecef0, this_v_ecef0);
  double that_energy = energy(that_r_ecef0, that_v_ecef0);

  // Hill frame PD controller
  lin::Vector3d r_hill = (DCM_hill_ecef0 * (this_r_ecef0 - that_r_ecef0)).eval();
  lin::Vector3d v_hill = (DCM_hill_ecef0 * (this_v_ecef0 - that_v_ecef0) + lin::cross(w_earth, r_hill)).eval();
  double p_term = K_p * r_hill(1);
  double d_term = -K_d * v_hill(1);
  lin::Vector3d dv_p = p_term * this_v_hat;
  lin::Vector3d dv_d = d_term * this_v_hat; 
    
  // Energy controller
  double e_term = -K_e * (this_energy - that_energy); // multiply by -1
  lin::Vector3d dv_e = e_term * this_v_hat;

  // H Controller

  // Define the direction of our impulse, always in the h2 direction
  lin::Vector3d Jhat_plane = this_h_hat;

  // Project h1 onto the plane formed by h2 and (r2 x h2)
  lin::Vector3d that_h_proj = that_h_ecef0 - lin::dot(that_h_ecef0, this_r_hat) * this_r_hat;

  // Calculate the angle between h1proj and h2 (this is what we are driving to zero with this burn)
  double theta = lin::atan2( lin::dot(that_h_proj, lin::cross(this_r_ecef0, this_h_ecef0)), lin::dot(that_h_proj, this_h_ecef0) );

  // Scale the impulse delivered by the angle theta
  lin::Vector3d J_plane = theta * Jhat_plane;

  // Scale dv by h_gain
  lin::Vector3d dv_plane = K_h * J_plane / mass;

  // Total dv to be applied from all controllers
  lin::Vector3d dv = dv_p + dv_d + dv_e + dv_plane;

  // Thruster saturation
  if (lin::norm(dv) > max_dv) {
    dv = max_dv * (dv / lin::norm(dv));
  }
  if (lin::norm(dv) < min_dv) {
    dv = min_dv * (dv / lin::norm(dv));
  }

  // Apply actuation output
  actuation.J_ecef = mass * dv;
  actuation.phase_till_next_node = 0;
  // How do I update actuate.phase_until_next_node?

}

#ifndef MEX
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation) {
  mex_control_orbit(state, data, actuation, mass, p, d, energy_gain, h_gain);
}
#endif

}  // namespace gnc
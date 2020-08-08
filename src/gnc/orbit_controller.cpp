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

namespace gnc {

// Initialize constants
static constexpr double mass = 3.7;             // Mass of spacecraft                    (kg)
static constexpr double J_min = 0;              // Minimum impulse                       (N s)
static constexpr double J_max = 2500;           // Maximum impulse                       (N s)
static constexpr double max_dv = J_max / mass;  // Max velocity change                   (m/s)
static constexpr double min_dv = J_min / mass;  // Min velocity change                   (m/s)
static constexpr double t_drift = 60.0 * 60.0;  // Drift time                            (s)
static constexpr double p = 1.0e-6;
static constexpr double d = 5.0e-2;
static constexpr double energy_gain = 5.0e-5;   // Energy gain                           (J)
static constexpr double h_gain = 2.0e-3;        // Angular momentum gain                 (kg m^2/sec)
static constexpr double thrust_noise_ratio = 0; //
static constexpr double dt_fire_min = 5 * 60;   // Minimum time between firings          (sec)

// Initial orbital elements
static constexpr double a  = 6793137.0;         // Semimajor axis                        (m)
static constexpr double e  = 0.001;             // Eccentricity                          (unitless) - there's an "earth's eccentricity" value in constants.hpp so idk if this needs to change
static constexpr double I  = 45*pi/180;         // Inclination angle                     (rad)
static constexpr double O  = 0.0;               // Right ascension of the ascending node (rad)
static constexpr double o  = 0.0;               // Argument of perigee                   (rad)
static constexpr double nu = 0*pi/180;          // True anamoly                          (rad)

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
 * Quaternion frame rotation
 * Rotates the frame of reference for the three vector V
 * using quaternion Q stored as a vector with 4th component real. 
 */
static double rotate_frame(lin::Vector3d const &v, lin::Vector4d const &q) {
  // not familiar with lin library, so this is def gonna need to be fixed
  return v + lin::cross(2*q[1:3], lin::cross(q[1:3], v) - q[4] * v);
}

/*
 * Calculates orbital energy given position and velocity
 */
static double energy(lin::Vector3d const &r_eci, lin::Vector3d const &v_eci) {
  // Convert position from ECI to ECEF by rotating frame. not sure how to do this yet...
  lin::Vector3d r_ecef = r_eci;
  // lin::Vector3d v_ecef = v_eci + lin::cross(w_earth, r_eci);

  // Calculate gravitational potential
  double potential;
  lin::Vector3d acceleration;
  env::gravity(r_ecef, acceleration, potential);

  // Return energy (E = K + U)
  return 0.5 * lin::dot(v_eci, v_eci) - potential;
}

/*
 * Calculates the orbital rate of the satellite assuming a circular orbit
 * with the given semimajor axis a.
 */
static double orbrate(double a) {
  return std::sqrt(gnc::constant::mu_earth / (a^3));
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
  lin::Vector3d &that_h_hat;
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
    that_h_hat = that_h_ecef0 / lin::norm(that_h_ecef0);
  }

  // Calculate the hill frame DCM
  utl::dcm(DCM_hill_ecef0, that_r_ecef0, that_v_ecef0);

  // Calculate energy
  double this_energy = energy(this_r_ecef0, this_v_ecef0);
  double that_energy = energy(that_r_ecef0, that_v_ecef0);

  // Requested delta v gain along this_v_hat in ECEF0
  double dv_gain_v_hat =
      K_p * (DCM_hill_ecef0 * (that_r_ecef0 - this_r_ecef0))(1) +
      K_d * (DCM_hill_ecef0 * (that_v_ecef0 - this_v_ecef0))(1) +
      K_e * this_energy - that_energy;

  // Calculate follower orbital elements. may have to switch reference frame for energy calculation
  double a = -1 * gnc::constant::mu_earth / (2 * that_energy);
  double n = orbrate(a);
  double M = n * data.t;
  M = M % (2 * gnc::constant::pi);

  // Calculate eccentric anomaly
  double del = 1;
  double E = M / (1 - e);

  // Check if follower is at a firing point
  if (E % (gnc::constant::pi / 4) < 0.01 && data.t - state.t_last_firing > dt_fire_min) {
    // Record firing time and position
    state.t_last_firing = data.t;
    // r_fire = [r_fire, [r1; r2]]; instead of r_fire, i guess it would be state.this_r_ecef0???

    // Hill frame PD controller
    double pterm = K_p * r_hill(2);
    double dterm = -1 * K_d * v_hill(2);
    double dv_p = pterm * that_v_ecef0 / lin::norm(that_v_ecef0);
    double dv_d = dterm * that_v_ecef0 / lin::norm(that_v_ecef0);

    // Energy controller
    double energy_term = -1 * K_e * (that_energy - this_energy);
    double dv_energy = energy_term * v2 / lin::norm(that_v_ecef0);

    // H controller
    that_r_hat = that_r_ecef0 / lin::norm(that_r_ecef0);

    // Define the direction of our impulse, always in the h2 direction
    lin::Vector3d Jhat_plane = that_h_hat;

    // Project h1 onto the plane formed by h2 and (r2 x h2)
    lin::Vector3d this_h_proj = this_h_ecef0 - lin::dot(this_h_ecef0, that_r_hat) * that_r_hat;

    // Calculate the angle between h1proj and h2 (this is what we are driving to zero with this burn)
    double theta = lin::atan( lin::dot(this_h_proj, lin::cross(that_r_ecef0, that_h_ecef0)) / lin::dot(this_h_proj, that_h_ecef0) );

    // Scale the impulse delivered by the angle theta
    lin::Vector3d J_plane = theta * Jhat_plane;

    // Scale dv by h_gain
    dv_plane = h_gain * J_plane / mass;

    // Total dv to be applied from all controllers
    dv = dv_p + dv_d + dv_energy + dv_plane;

    // Thruster saturation

    // Apply dv
  }

  // A few of the lines here are repeated calculations
  lin::Vector3d dv;
  {
    lin::Vector3d this_v_hat = this_v_ecef0 / lin::norm(this_v_ecef0); // why is this calculated twice?

    dv = (K_p * r_hill(1) +
          K_d * v_hill(1) +
          K_e * (energy(this_r_ecef0, this_v_ecef0) - energy(that_r_ecef0, that_v_ecef0))
        ) * this_v_hat; // very similar to calculation for dv_gain_v_hat. what's the difference?

    lin::Vector3d this_r_hat = this_r_ecef0 / lin::norm(this_r_ecef0); // why is this calculated twice?
    lin::Vector3d this_h_hat = lin::cross(this_r_hat, this_v_hat); // how is this different from: this_h_hat = this_h_ecef0 / lin::norm(this_h_ecef0); ?

    lin::Vector3d that_h_proj = lin::cross(that_r_ecef0, that_v_ecef0); // repeated calculation of that_h_ecef0
    that_h_proj = that_h_proj - lin::dot(that_h_proj, this_r_hat) * this_r_hat;

    double theta = lin::atan(lin::dot(that_h_proj, lin::cross(this_r_ecef0, this_h)) / ); // what does theta represent? why is this line incomplete?
  }

}

// Spacecraft mass

#ifndef MEX
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation) {

}
#endif

}  // namespace gnc
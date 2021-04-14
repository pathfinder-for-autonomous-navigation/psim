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
 *  @author Fatima Yousuf
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

namespace gnc {

// Initialize constants
static constexpr double mass = 3.7;             // Mass of spacecraft            (kg)
static constexpr double J_min = 0;              // Minimum impulse               (N s)
static constexpr double J_max = 2.5e-2;         // Maximum impulse               (N s)
static constexpr double max_dv = J_max / mass;  // Max velocity change           (m/s)
static constexpr double min_dv = J_min / mass;  // Min velocity change           (m/s)

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
  : J_ecef(lin::nans<decltype(J_ecef)>()) { }

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
void mex_control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation,
    double mass) {
  // Default actuation outputs
  actuation = OrbitActuation();

  

  // Pull in references to the controller's state entries
  auto K_p = data.p;
  auto K_d = data.d;
  auto K_e = data.energy_gain;  // Energy gain
  auto K_h = data.h_gain;       // Angular momentum gain

  auto &this_r_ecef0 = state.this_r_ecef0;
  auto &that_r_ecef0 = state.that_r_ecef0;
  auto &this_r_hat = state.this_r_hat;

  auto &this_v_ecef0 = state.this_v_ecef0;
  auto &that_v_ecef0 = state.that_v_ecef0;
  auto &this_v_hat = state.this_v_hat;

  auto &this_h_ecef0 = state.this_h_ecef0;
  auto &that_h_ecef0 = state.that_h_ecef0;
  auto &this_h_hat = state.this_h_hat;

  auto &DCM_hill_ecef0 = state.DCM_hill_ecef0;

  // Move to the inertial ECEF0 frame
  {
    lin::Vector3f w_earth;
    env::earth_angular_rate(data.t, w_earth);

    // Position in ECEF and ECEF0 are the same
    this_r_ecef0 = data.r_ecef;
    that_r_ecef0 = data.r_ecef + data.dr_ecef;
    this_r_hat = this_r_ecef0 / lin::norm(this_r_ecef0);

    // Velocities need to velocity of the ECEF frame itself added on
    this_v_ecef0 = data.v_ecef - lin::cross(w_earth, this_r_ecef0);
    that_v_ecef0 = data.v_ecef + data.dv_ecef - lin::cross(w_earth, that_r_ecef0);
    this_v_hat = this_v_ecef0 / lin::norm(this_v_ecef0);

    // Calculate the satellites angular momentums (without scaling by mass)
    this_h_ecef0 = lin::cross(this_r_ecef0, this_v_ecef0);
    that_h_ecef0 = lin::cross(that_r_ecef0, that_v_ecef0);
    this_h_hat = this_h_ecef0 / lin::norm(this_h_ecef0);
  }

  // Calculate the DCM rotating from ECEF0 to the hill frame
  utl::dcm(DCM_hill_ecef0, that_r_ecef0, that_v_ecef0);

  // In-plane controller
  lin::Vector3d dv_in_plane;
  {
    // Calculate orbital energies
    double const this_energy = energy(this_r_ecef0, this_v_ecef0);
    double const that_energy = energy(that_r_ecef0, that_v_ecef0);

    // Angular rate of that satellite's hill frame
    lin::Vector3d const w_hill = that_h_ecef0 / lin::fro(that_r_ecef0);

    // Position and velocity of this satellite in the other's hill frame
    lin::Vector3d const r_hill = DCM_hill_ecef0 * (this_r_ecef0 - that_r_ecef0).eval();
    lin::Vector3d const v_hill = DCM_hill_ecef0 * (this_v_ecef0 - that_v_ecef0).eval() - lin::cross(w_hill, r_hill);

    // Hill frame PD controller
    dv_in_plane = this_v_hat * (
        ( K_p * r_hill(1)) +                  // Hill position term
        (-K_d * v_hill(1)) +                  // Hill velocity term
        (-K_e * (this_energy - that_energy))  // Energy term
      );
  }

  // Angular momentum controller (matching orbital planes)
  lin::Vector3d dv_plane;
  {
    // Project that satellite's angular momentum onto the plane perpendicular
    // to this satellite's position vector
    lin::Vector3d that_h_proj = that_h_ecef0 - lin::dot(that_h_ecef0, this_r_hat) * this_r_hat;

    // Calculate the angle between this projection of the other satellites
    // angular momentum and our angular momentum
    double const theta = lin::atan2(
        lin::dot(that_h_proj, lin::cross(this_r_ecef0, this_h_ecef0)),
        lin::dot(that_h_proj, this_h_ecef0)
      ); // ^^ this seems a tad weird to me and would be of very different
         // orders of magnitude - i.e. `this_r_ecef0` or `this_r_hat`?

    dv_plane = this_h_hat * (K_h * theta / mass);
    // Don't understand why we divide by mass ^^ as we never were working with
    // orbital angular momentum anyway
  }

  // Calculate final dv command and account for saturation events
  lin::Vector3d dv;
  {
 /   dv = dv_plane + dv_in_plane;

    auto const fro_dv = lin::fro(dv);
    if (fro_dv > max_dv * max_dv)
      dv = max_dv * (dv / lin::sqrt(fro_dv));
    else if (fro_dv < min_dv * min_dv)
      dv = min_dv * (dv / lin::sqrt(fro_dv));
  }

  actuation.J_ecef = mass * dv;
}

#ifndef MEX
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation) {
  mex_control_orbit(state, data, actuation, mass);
}
#endif

}  // namespace gnc

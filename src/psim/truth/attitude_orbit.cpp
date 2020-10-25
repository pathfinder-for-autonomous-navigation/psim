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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/truth/orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/attitude_orbit.hpp>

#include <gnc/environment.hpp>
#include <gnc/utilities.hpp> //useful functions, like cross product

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

namespace psim {
 
AttitudeOrbitNoFuelGnc::AttitudeOrbitNoFuelGnc(Configuration const &config,
    std::string const &prefix, std::string const &satellite)
  : Super(config, prefix, satellite, "eci") { }


struct IntegratorData{
  Real const &mass;
  Vector3 const &I;
  Real const &I_w;
  Vector3 const &tau_w;
  Vector3 const &m;
  Vector3 const &b;
};

void AttitudeOrbitNoFuelGnc::step() {
  this->Super::step();

  // Initialize variables from .yml
  auto &mass = prefix_satellite_m.get();
  auto &I = prefix_satellite_J.get();
  auto &I_w = prefix_satellite_wheels_J.get();
  auto &tau_w = prefix_satellite_wheels_t.get();
  auto &omega_w = prefix_satellite_wheels_w.get();
  auto &m = prefix_satellite_magnetorquers_m.get();
  auto &b = prefix_satellite_environment_b_eci->get();

  auto &q_body_eci = prefix_satellite_attitude_q_body_eci.get();
  auto &w = prefix_satellite_attitude_w.get();

  // References to the current time and timestep
  auto const &dt = prefix_dt_s->get();
  auto const &t = prefix_t_s->get();

  // References to position and velocity
  auto &r = prefix_satellite_orbit_r.get();
  auto &v = prefix_satellite_orbit_v.get();

  IntegratorData data {mass, I, I_w, tau_w, m, b};

  // Simulate our dynamics
  
  lin::Vector<Real, 16> const x = {
      r(0), r(1), r(2),
      v(0), v(1), v(2),
      q_body_eci(0), q_body_eci(1), q_body_eci(2), q_body_eci(3),
      w(0), w(1), w(2),
      omega_w(0), omega_w(1), omega_w(2)
  };

  auto const xf = ode(t, dt, x, &data,
      [](Real t, lin::Vector<Real, 16> const &x, void *ptr) -> lin::Vector<Real, 16> { // include/gnc/ode4.hpp
    // position, velocity, quat_body, ang velocity, wheel ang velocity
    auto const r = lin::ref<3, 1>(x, 0,  0);
    auto const v = lin::ref<3, 1>(x, 3,  0);
    auto const q = lin::ref<4, 1>(x, 6,  0);
    auto const w = lin::ref<3, 1>(x, 10, 0);
    auto const w_w = lin::ref<3, 1>(x, 14, 0);
    auto *data = (IntegratorData *) ptr;

    lin::Vector<Real, 16> dx;

    // Orbital dynamics
    {
      // Calculate the Earth's current attitude (Position in ECI -> gravitational acceleration vector in ECI)
      Vector4 q; 
      gnc::env::earth_attitude(t, q);  // q = q_ecef_eci

      // Determine our gravitation acceleration in ECI
      Vector3 g;
      {
        Vector3 r_ecef;
        gnc::utl::rotate_frame(q, r.eval(), r_ecef);

        Real _;
        gnc::env::gravity(r_ecef, g, _);  // g = g_ecef
        gnc::utl::quat_conj(q);           // q = q_eci_ecef
        gnc::utl::rotate_frame(q, g);     // g = g_eci
      }

      lin::ref<6, 1>(dx, 0, 0) = {v(0), v(1), v(2), g(0), g(1), g(2)};
    }
    
    // dq = utl_quat_cross_mult(0.5*quat_rate,quat_body_eci);
    Vector4 dq;
    Vector4 quat_rate = {0.5 * w(0), 0.5 * w(1), 0.5 * w(2), 0.0};
    gnc::utl::quat_cross_mult(quat_rate, q.eval(), dq);

    // dw = I^{-1} * m x b - tau_w - w x (I * w + I_w * w_w)
    Vector3 dw = lin::cross(data->m,data->b) - data->tau_w - lin::cross(w, lin::multiply(data->I, w) + (data->I_w)*w_w);
    dw = lin::divide(dw, data->I); // Multiplication by I inverse

    // dw_w = tau_w/I_w
    Vector3 dw_w = lin::divide(data->tau_w, data->I_w);


    // Attitude dynamics
    {
      lin::ref<4, 1>(dx, 6, 0) = dq; 
      lin::ref<3, 1>(dx, 10, 0) = dw; 
      lin::ref<3, 1>(dx, 14, 0) = dw_w;
    }

    return dx;
  }); 

  // Write back to our state fields
  r = lin::ref<3, 1>(xf, 0, 0);
  v = lin::ref<3, 1>(xf, 3, 0);
  q_body_eci = lin::ref<4, 1>(xf, 6, 0);
  w = lin::ref<3, 1>(xf, 10, 0);
  omega_w = lin::ref<3, 1>(xf, 14, 0);
}
 
}  // namespace psim

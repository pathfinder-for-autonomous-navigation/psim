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
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

#include <psim/truth/orbit_utilities.hpp>

namespace psim {

AttitudeOrbitNoFuelEcef::AttitudeOrbitNoFuelEcef(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite)
  : Super(randoms, config, satellite, "ecef") {}

void AttitudeOrbitNoFuelEcef::step() {
  this->Super::step();

  struct IntegratorData {
    Real const &m;
    Vector3 const &J_body;
    Vector3 const &wheels_J_body;
    Vector3 const &wheels_t_body;
    Vector3 const &m_body;
    Vector3 const &b_eci;
    Vector3 const &earth_w;
    Vector3 const &earth_w_dot;
  };

  auto const &dt = truth_dt_s->get();
  auto const &earth_w = truth_earth_w->get();
  auto const &earth_w_dot = truth_earth_w_dot->get();
  auto const &m = truth_satellite_m.get();
  auto const &J_body = truth_satellite_J.get();
  auto const &wheels_J_body = truth_satellite_J.get();
  auto const &wheels_t_body = truth_satellite_wheels_t.get();
  auto const &b_eci = truth_satellite_environment_b_eci->get();

  auto &r_ecef = truth_satellite_orbit_r.get();
  auto &v_ecef = truth_satellite_orbit_v.get();
  auto &J_ecef = truth_satellite_orbit_J_frame.get();
  auto &q_body_eci = truth_satellite_attitude_q_body_eci.get();
  auto &w_body = truth_satellite_attitude_w.get();
  auto &wheels_w_body = truth_satellite_wheels_w.get();
  auto &m_body = truth_satellite_magnetorquers_m.get();

  // Thruster firings are modelled here as instantaneous impulses. This removes
  // thruster dependance from the state dot function in the integrator.
  v_ecef = v_ecef + J_ecef / m;
  J_ecef = lin::zeros<Vector3>();

  // Prepare integrator inputs.
  Vector<16> x;
  lin::ref<Vector3>(x, 0, 0) = r_ecef;
  lin::ref<Vector3>(x, 3, 0) = v_ecef;
  lin::ref<Vector4>(x, 6, 0) = q_body_eci;
  lin::ref<Vector3>(x, 10, 0) = w_body;
  lin::ref<Vector3>(x, 13, 0) = wheels_w_body;
  IntegratorData data{m, J_body, wheels_J_body, wheels_t_body, m_body, b_eci,
      earth_w, earth_w_dot};

  // Simulate dynamics.
  x = ode(Real(0.0), dt, x, &data,
      [](Real t, Vector<16> const &x, void *ptr) -> Vector<16> {
        auto const *data = static_cast<IntegratorData *>(ptr);

        auto const &m = data->m;
        auto const earth_w = (data->earth_w + t * data->earth_w_dot).eval();
        auto const &earth_w_dot = data->earth_w_dot;

        auto const r_ecef = lin::ref<Vector3>(x, 0, 0);
        auto const v_ecef = lin::ref<Vector3>(x, 3, 0);
        auto const q_body_eci = lin::ref<Vector4>(x, 6, 0);
        auto const w_body = lin::ref<Vector3>(x, 10, 0);
        auto const wheels_w_body = lin::ref<Vector3>(x, 13, 0);

        auto const S = 0.0; [&q_body_eci]() {
          
        }();
        auto const b_body = [&q_body_eci](Vector3 const &b_eci) {
          Vector3 b_body;
          gnc::utl::rotate_frame(q_body_eci.eval(), b_eci, b_body);
          return b_body;
        }(data->b_eci);

        Vector<16> dx;

        // Orbital dynamics
        {
          Vector3 a_ecef;
          orbit::acceleration(
              earth_w, earth_w_dot, r_ecef.eval(), v_ecef.eval(), S, m, a_ecef);

          lin::ref<Vector3>(dx, 0, 0) = v_ecef;
          lin::ref<Vector3>(dx, 3, 0) = a_ecef;
        }

        // dq = utl_quat_cross_mult(0.5*quat_rate,quat_body_eci);
        Vector4 dq;
        Vector4 quat_rate = {0.5 * w(0), 0.5 * w(1), 0.5 * w(2), 0.0};
        gnc::utl::quat_cross_mult(quat_rate, q.eval(), dq);

        // dw = I^{-1} * m x b - tau_w - w x (I * w + I_w * w_w)
        Vector3 dw =
            lin::cross(data->m, data->b) - data->tau_w -
            lin::cross(w, lin::multiply(data->I, w) + (data->I_w) * w_w);
        dw = lin::divide(dw, data->I); // Multiplication by I inverse

        // dw_w = tau_w/I_w
        Vector3 dw_w = lin::divide(data->tau_w, data->I_w);

        // Attitude dynamics
        {
          lin::ref<Vector4>(dx, 6, 0) = dq;
          lin::ref<Vector3>(dx, 10, 0) = dw;
          lin::ref<Vector3>(dx, 13, 0) = dw_w;
        }

        return dx;
      });

  // Write back to our state fields
  r_ecef = lin::ref<Vector3>(x, 0, 0);
  v_ecef = lin::ref<Vector3>(x, 3, 0);
  q_body_eci = lin::ref<Vector4>(x, 6, 0);
  w_body = lin::ref<Vector3>(x, 10, 0);
  wheels_w_body = lin::ref<Vector3>(x, 13, 0);
}

Vector4 AttitudeOrbitNoFuelEcef::truth_satellite_attitude_q_eci_body() const {
  auto const &q_body_eci = this->truth_satellite_attitude_q_body_eci.get();

  Vector4 q_eci_body;
  gnc::utl::quat_conj(q_body_eci, q_eci_body);
  return q_eci_body;
}

Vector3 AttitudeOrbitNoFuelEcef::truth_satellite_attitude_L() const {
  auto const &J = truth_satellite_J.get();
  auto const &J_w = truth_satellite_wheels_J.get();
  auto const &w = truth_satellite_attitude_w.get();
  auto const &wheels_w = truth_satellite_wheels_w.get();

  return lin::multiply(J, w) + J_w * wheels_w;
}
} // namespace psim

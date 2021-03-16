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

#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

#include <psim/truth/attitude_utilities.hpp>
#include <psim/truth/orbit_utilities.hpp>

namespace psim {

AttitudeOrbitNoFuelEcef::AttitudeOrbitNoFuelEcef(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite)
  : Super(randoms, config, satellite, "ecef") {}

void AttitudeOrbitNoFuelEcef::step() {
  this->Super::step();

  struct IntegratorData {
    Real const &m;
    Real const &S;
    Vector3 const &earth_w;
    Vector3 const &earth_w_dot;
    Vector3 const &J_body;
    Real const &wheels_J_body;
    Vector3 const &wheels_t_body;
    Vector3 const &m_body;
    Vector3 const &b_eci;
  };

  auto const &dt = truth_dt_s->get();
  auto const &earth_w = truth_earth_w->get();
  auto const &earth_w_dot = truth_earth_w_dot->get();
  auto const &q_eci_ecef = truth_earth_q_eci_ecef->get();
  auto const &m = truth_satellite_m.get();
  auto const &J_body = truth_satellite_J.get();
  auto const &wheels_J_body = truth_satellite_wheels_J.get();
  auto const &wheels_t_body = truth_satellite_wheels_t.get();
  auto const &b_eci = truth_satellite_environment_b_eci->get();

  auto &S = truth_satellite_S.get();
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

  // Calculate surface area projected along the direction of travel. It's
  // this is constant over the course of a timestep.
  S = attitude::S(q_body_eci, q_eci_ecef, v_ecef);

  // Prepare integrator inputs.
  Vector<16> x;
  lin::ref<Vector3>(x, 0, 0) = r_ecef;
  lin::ref<Vector3>(x, 3, 0) = v_ecef;
  lin::ref<Vector4>(x, 6, 0) = q_body_eci;
  lin::ref<Vector3>(x, 10, 0) = w_body;
  lin::ref<Vector3>(x, 13, 0) = wheels_w_body;
  IntegratorData data{m, S, earth_w, earth_w_dot, J_body, wheels_J_body,
      wheels_t_body, m_body, b_eci};

  // Simulate dynamics.
  x = ode(Real(0.0), dt, x, &data,
      [](Real t, Vector<16> const &x, void *ptr) -> Vector<16> {
        auto const *data = static_cast<IntegratorData *>(ptr);

        auto const &m = data->m;
        auto const &S = data->S;
        auto const earth_w = (data->earth_w + t * data->earth_w_dot).eval();
        auto const &earth_w_dot = data->earth_w_dot;
        auto const &J_body = data->J_body;
        auto const &wheels_J_body = data->wheels_J_body;
        auto const &wheels_t_body = data->wheels_t_body;
        auto const &m_body = data->m_body;

        auto const r_ecef = lin::ref<Vector3>(x, 0, 0);
        auto const v_ecef = lin::ref<Vector3>(x, 3, 0);
        auto const q_body_eci = lin::ref<Vector4>(x, 6, 0);
        auto const w_body = lin::ref<Vector3>(x, 10, 0);
        auto const wheels_w_body = lin::ref<Vector3>(x, 13, 0);
        auto const b_body = [&q_body_eci](Vector3 const &b_eci) {
          Vector3 b_body;
          gnc::utl::rotate_frame(q_body_eci.eval(), b_eci, b_body);
          return b_body;
        }(data->b_eci);

        Vector<16> dx;

        // Orbital dynamics
        {
          Vector3 const a_ecef = orbit::acceleration(
              earth_w, earth_w_dot, r_ecef.eval(), v_ecef.eval(), S, m);

          lin::ref<Vector3>(dx, 0, 0) = v_ecef;
          lin::ref<Vector3>(dx, 3, 0) = a_ecef;
        }

        // Attitude dynamics - quaternion
        {
          Vector4 dq_body_eci;
          Vector4 const dq = {
              0.5 * w_body(0), 0.5 * w_body(1), 0.5 * w_body(2), 0.0};
          gnc::utl::quat_cross_mult(dq, q_body_eci.eval(), dq_body_eci);

          lin::ref<Vector4>(dx, 6, 0) = dq_body_eci;
        }

        // Attitude dynamics - angular rate
        {
          /* The total angular momentum of the spacecraft is given by:
           *
           *   H = J * w + J_wheels * w_wheels
           *
           * which allows us to represent Euler's rotation equation as:
           *
           *   J alpha = mu x b - tau_wheels - w x H.
           * 
           * Recall that the torque commanded to the wheels exerts the opposite
           * of that on the spacecraft itself.
           *
           * Reference(s):
           *  - https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
           *  - https://en.wikipedia.org/wiki/Magnetic_moment
           */
          Vector3 const H_body =
              lin::multiply(J_body, w_body) + wheels_J_body * wheels_w_body;
          Vector3 const t_body = lin::cross(m_body, b_body) - wheels_t_body -
                                 lin::cross(w_body, H_body);
          Vector3 const dw_body = lin::divide(t_body, J_body);

          lin::ref<Vector3>(dx, 10, 0) = dw_body;
        }

        // Attitude dynamics - reaction wheel rates
        {
          Vector3 const dwheels_w_body = wheels_t_body / wheels_J_body;

          lin::ref<Vector3>(dx, 13, 0) = dwheels_w_body;
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

Real AttitudeOrbitNoFuelEcef::truth_satellite_orbit_altitude() const {
  auto const &r_ecef = truth_satellite_orbit_r.get();

  return lin::norm(r_ecef) - gnc::constant::r_earth;
}

Vector3 AttitudeOrbitNoFuelEcef::truth_satellite_orbit_a_gravity() const {
  auto const &r_ecef = truth_satellite_orbit_r.get();

  return orbit::gravity(r_ecef);
}

Vector3 AttitudeOrbitNoFuelEcef::truth_satellite_orbit_a_drag() const {
  auto const &m = truth_satellite_m.get();
  auto const &S = truth_satellite_S.get();
  auto const &r_ecef = truth_satellite_orbit_r.get();
  auto const &v_ecef = truth_satellite_orbit_v.get();

  return orbit::drag(r_ecef, v_ecef, S, m);
}

Vector3 AttitudeOrbitNoFuelEcef::truth_satellite_orbit_a_rot() const {
  auto const &earth_w = truth_earth_w->get();
  auto const &earth_w_dot = truth_earth_w_dot->get();
  auto const &r_ecef = truth_satellite_orbit_r.get();
  auto const &v_ecef = truth_satellite_orbit_v.get();

  return orbit::rotational(earth_w, earth_w_dot, r_ecef, v_ecef);
}

Real AttitudeOrbitNoFuelEcef::truth_satellite_orbit_density() const {
  auto const &r_ecef = truth_satellite_orbit_r.get();

  return orbit::density(r_ecef);
};

Real AttitudeOrbitNoFuelEcef::truth_satellite_orbit_T() const {
  static constexpr Real half = 0.5;

  auto const &earth_w = truth_earth_w->get();
  auto const &r_ecef = truth_satellite_orbit_r.get();
  auto const &v_ecef = truth_satellite_orbit_v.get();
  auto const &m = truth_satellite_m.get();

  return half * m * lin::fro(v_ecef + lin::cross(earth_w, r_ecef));
}

Real AttitudeOrbitNoFuelEcef::truth_satellite_orbit_U() const {
  auto const &r_ecef = truth_satellite_orbit_r.get();
  auto const &m = truth_satellite_m.get();

  Real U;
  orbit::gravity(r_ecef, U);

  return m * U;
}

Real AttitudeOrbitNoFuelEcef::truth_satellite_orbit_E() const {
  auto const &T = this->Super::truth_satellite_orbit_T.get();
  auto const &U = this->Super::truth_satellite_orbit_U.get();

  return T - U;
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

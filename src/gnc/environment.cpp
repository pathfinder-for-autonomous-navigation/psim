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

/** @file gnc_environment.cpp
 *  @author Kyle Krol
 */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <orb/geograv.hpp>
#include <orb/GGM05S.hpp>

#include <cmath>

#include "inl/geomag.hpp"

namespace gnc {
namespace env {

void earth_attitude(double t, lin::Vector4d &q_ecef_eci) {
  // Determine a transformation from ecef0 to ecef0p
  lin::Vector4d q_ecef0p_ecef0({  // Small angle approx sin(x) ~ x
    t * constant::earth_precession_rate(0),
    t * constant::earth_precession_rate(1),
    t * constant::earth_precession_rate(2),
    1.0
  });
  q_ecef0p_ecef0 = q_ecef0p_ecef0 / lin::norm(q_ecef0p_ecef0);
  // Earth's rotation angle about the z axis (assumes x, y components are zero)
  double theta = t * constant::earth_rate_ecef_z;
  lin::Vector4d q_ecef_ecef0p({
    0.0,
    0.0,
    std::sin(theta / 2.0),
    std::cos(theta / 2.0)
  });
  // Rotate through ECI -> ECEF0 -> ECEF0P -> ECEF
  lin::Vector4d temp;
  utl::quat_cross_mult(q_ecef0p_ecef0, constant::q_ecef0_eci, temp);
  utl::quat_cross_mult(q_ecef_ecef0p, temp, q_ecef_eci);
}

void earth_attitude(double t, lin::Vector4f &q_ecef_eci) {
  lin::Vector4d _q_ecef_eci;
  earth_attitude(t, _q_ecef_eci);
  q_ecef_eci = _q_ecef_eci;
}

void earth_angular_rate(double t, lin::Vector3d &w_eci) {
  w_eci = { 0.0, 0.0, constant::earth_rate_ecef_z };
}

void earth_angular_rate(double t, lin::Vector3f &w_eci) {
  w_eci = { 0.0, 0.0, constant::earth_rate_ecef_z };
}

void gravity(lin::Vector3d const &r_ecef, lin::Vector3f &g_ecef, float &U_ecef) {
  double _U_ecef;
  lin::Vector3d _g_ecef;
  gravity(r_ecef, _g_ecef, _U_ecef);
  g_ecef = _g_ecef;
  U_ecef = _U_ecef;
}

void gravity(lin::Vector3d const &r_ecef, lin::Vector3d &g_ecef, double &U_ecef) {
  // Setup the gravity model at compile time
  GNC_TRACKED_CONSTANT(constexpr static int, env_grav_order, 4);
  constexpr static geograv::Coeff<env_grav_order> env_grav_model = static_cast<geograv::Coeff<env_grav_order>>(GGM05S);

  // Make a call to the model
  geograv::Vector in;
  geograv::Vector g;
  in.x = r_ecef(0);
  in.y = r_ecef(1);
  in.z = r_ecef(2);
  U_ecef = geograv::GeoGrav(in, g, env_grav_model, true);
  g_ecef = { g.x, g.y, g.z };
}

// t will never get large enough for floating point precession to be an issue
void sun_vector(double t, lin::Vector3f &s_eci) {
  // Calculate Earth's position in the perifocal frame
  float E = static_cast<float>(constant::two_pi) * (static_cast<float>(t) -
      static_cast<float>(constant::earth_perihelion_time)) / static_cast<float>(constant::earth_period);
  E = E + static_cast<float>(constant::earth_eccentricity) * sinf(E);
  s_eci = {  // Negative of Earth's position (want to point at the Sun)
    static_cast<float>(constant::earth_eccentricity) - cosf(E),
    sinf(E) * (0.5f * static_cast<float>(constant::earth_eccentricity) * static_cast<float>(constant::earth_eccentricity) - 1.0f),
    0.0f
  };
  s_eci = s_eci / lin::norm(s_eci);  // Puts us in AU (approximately) and is a normalized vector
  // Rotate into ECI
  lin::Vector4f q_eci_perifocal_f = constant::q_eci_perifocal;
  utl::rotate_frame(q_eci_perifocal_f, s_eci);
}

void sun_vector(double t, lin::Vector3d &s_eci) {
  lin::Vector3f _s_eci;
  sun_vector(t, _s_eci);
  s_eci = _s_eci;
}

void magnetic_field(double t, lin::Vector3d const &r_ecef, lin::Vector3f &b_ecef) {
  geomag::Vector in, out;
  in.x = r_ecef(0);
  in.y = r_ecef(1);
  in.z = r_ecef(2);
  out = geomag::GeoMag(constant::init_dec_year + t / (365.0 * 24.0 * 60.0 * 60.0),in, geomag::WMM2020);
  b_ecef = { out.x, out.y, out.z };
}

void magnetic_field(double t, lin::Vector3d const &r_ecef, lin::Vector3d &b_ecef) {
  lin::Vector3f _b_ecef;
  magnetic_field(t, r_ecef, _b_ecef);
  b_ecef = _b_ecef;
}
}  // namespace env
}  // namespace gnc

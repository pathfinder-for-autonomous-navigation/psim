//
// src/gnc_environment.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_constants.hpp>
#include <gnc_environment.hpp>
#include <gnc_utilities.hpp>

#include <cmath>

#include "inl/geomag.inl"

namespace gnc {
namespace env {

void earth_attitude(double t, lin::Vector4d &q) {
  // Determine a transformation from ecef0 to ecef0p
  lin::Vector4d q_ecef0p_ecef0({
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
    sin(theta / 2.0),
    cos(theta / 2.0)
  });
  // Rotate through ECI -> ECEF0 -> ECEF0P -> ECEF
  lin::Vector4d temp;
  utl::quat_cross_mult(q_ecef0p_ecef0, constant::q_ecef0_eci, temp);
  utl::quat_cross_mult(q_ecef_ecef0p, temp, q);
}

void earth_attitude(double t, lin::Vector4f &q) {
  lin::Vector4d _q;
  earth_attitude(t, _q);
  for (lin::size_t i = 0; i < q.size(); i++)
      q(i) = static_cast<float>(_q(i));
}

void earth_angular_rate(double t, lin::Vector3d &w) {
  w = { 0.0, 0.0, constant::earth_rate_ecef_z };
}

void earth_angular_rate(double t, lin::Vector3f &w) {
  w = { 0.0f, 0.0f, static_cast<float>(constant::earth_rate_ecef_z) };
}

void sun_vector(double t, lin::Vector3d &s) {
  // Calculate Earth's position in the perifocal frame
  double E = constant::two_pi * (t - constant::earth_perihelion_time) / constant::earth_period;
  E = E + constant::earth_eccentricity * sin(E);
  s = {  // Negative of Earth's position (want to point at the Sun)
    constant::earth_eccentricity - cos(E),
    sin(E) * (0.5 * constant::earth_eccentricity * constant::earth_eccentricity - 1.0),
    0.0
  };
  s = s / lin::norm(s);  // Puts us in AU (approximately) and is a normalized vector
  // Rotate into ECI
  utl::rotate_frame(constant::q_eci_perifocal, s);
}

void sun_vector(double t, lin::Vector3f &s) {
  lin::Vector3d _s;
  sun_vector(t, _s);
  for (lin::size_t i = 0; i < s.size(); i++)
    s(i) = static_cast<float>(_s(i));
}

void magnetic_field(double t, lin::Vector3f const &r, lin::Vector3f &b) {
  geomag::Vector in, out;
  in.x = r(0);
  in.y = r(1);
  in.z = r(2);
  out = geomag::GeoMag(constant::init_dec_year + t / (365.0 * 24.0 * 60.0 * 60.0),in, geomag::WMM);
  b = { out.x, out.y, out.z };
}
}  // namespace env
}  // namespace gnc

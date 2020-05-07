/** @file gnc_environment.cpp
 *  @author Kyle Krol */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <cmath>

#include "inl/geomag.hpp"

namespace gnc {
namespace env {

void earth_attitude(double t, lin::Vector4d &q) {
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
  w = { 0.0, 0.0, constant::earth_rate_ecef_z };
}

// t will never get large enough for floating point precession to be an issue
void sun_vector(double t, lin::Vector3f &s) {
  // Calculate Earth's position in the perifocal frame
  float E = static_cast<float>(constant::two_pi) * (static_cast<float>(t) -
      static_cast<float>(constant::earth_perihelion_time)) / static_cast<float>(constant::earth_period);
  E = E + static_cast<float>(constant::earth_eccentricity) * sinf(E);
  s = {  // Negative of Earth's position (want to point at the Sun)
    static_cast<float>(constant::earth_eccentricity) - cosf(E),
    sinf(E) * (0.5f * static_cast<float>(constant::earth_eccentricity) * static_cast<float>(constant::earth_eccentricity) - 1.0f),
    0.0f
  };
  s = s / lin::norm(s);  // Puts us in AU (approximately) and is a normalized vector
  // Rotate into ECI
  lin::Vector4f q_eci_perifocal_f = constant::q_eci_perifocal;
  utl::rotate_frame(q_eci_perifocal_f, s);
}

void sun_vector(double t, lin::Vector3d &s) {
  lin::Vector3f _s;
  sun_vector(t, _s);
  s = _s;
}

void magnetic_field(double t, lin::Vector3d const &r, lin::Vector3f &b) {
  geomag::Vector in, out;
  in.x = r(0);
  in.y = r(1);
  in.z = r(2);
  out = geomag::GeoMag(constant::init_dec_year + t / (365.0 * 24.0 * 60.0 * 60.0),in, geomag::WMM2020);
  b = { out.x, out.y, out.z };
}

void magnetic_field(double t, lin::Vector3d const &r, lin::Vector3d &b) {
  lin::Vector3f _b;
  magnetic_field(t, r, _b);
  b = _b;
}
}  // namespace env
}  // namespace gnc

/** @file gnc/constants.hpp
 *  Autocoded constants header file. See tools/constants_generator.py for more
 *  information. */

#ifndef GNC_CONSTANTS_HPP_
#define GNC_CONSTANTS_HPP_

#include "config.hpp"

#include <lin/core.hpp>

#include <cstdint>

#include <limits>

namespace gnc {
namespace constant {

GNC_TRACKED_CONSTANT(constexpr static double, pi, 3.141592653589793);

GNC_TRACKED_CONSTANT(constexpr static float, pi_f, pi);

GNC_TRACKED_CONSTANT(constexpr static double, two_pi, 2.0 * pi);

GNC_TRACKED_CONSTANT(constexpr static float, two_pi_f, two_pi);

GNC_TRACKED_CONSTANT(constexpr static double, deg_to_rad, pi / 180.0);

GNC_TRACKED_CONSTANT(constexpr static float, deg_to_rad_f, deg_to_rad);

GNC_TRACKED_CONSTANT(constexpr static double, rad_to_deg, 180.0 / pi);

GNC_TRACKED_CONSTANT(constexpr static float, rad_to_deg_f, rad_to_deg);

GNC_TRACKED_CONSTANT(constexpr static double, mu_earth, 3.986004418e14);

GNC_TRACKED_CONSTANT(constexpr static float, mu_earth_f, mu_earth);

GNC_TRACKED_CONSTANT(constexpr static double, nan, std::numeric_limits<double>::quiet_NaN());

GNC_TRACKED_CONSTANT(constexpr static float, nan_f, std::numeric_limits<float>::quiet_NaN());

extern unsigned short init_gps_week_number;

extern unsigned int init_gps_time_of_week;

extern unsigned long init_gps_nanoseconds;

extern double init_dec_year;

extern double earth_rate_ecef_z;

extern lin::Vector3d earth_precession_rate;

extern lin::Vector4d q_ecef0_eci;

GNC_TRACKED_CONSTANT(constexpr static double, earth_eccentricity, 0.0167086);

GNC_TRACKED_CONSTANT(constexpr static double, earth_perihelion_time, -6287982.0);

GNC_TRACKED_CONSTANT(constexpr static double, earth_period, 365.256363004 * 24.0 * 60.0 * 60.0);

GNC_TRACKED_CONSTANT(constexpr static lin::Vector4d, q_eci_perifocal, 0.127456632677880, -0.158124280715206, 0.762378784011859, -0.614434787689722);

GNC_TRACKED_CONSTANT(constexpr static float, b_noise_floor, 0.0f);

GNC_TRACKED_CONSTANT(constexpr static double, max_mtr_moment, 0.113337 / 2.0);

extern lin::Matrix3x3f J_sat;

extern float pointer_Kp;

extern float pointer_Kd;

GNC_TRACKED_CONSTANT(constexpr static float, J_wheel, 135.0e-7f);

GNC_TRACKED_CONSTANT(constexpr static float, w_wheel_max, 677.0f);

GNC_TRACKED_CONSTANT(constexpr static uint64_t, NANOSECONDS_IN_WEEK, 7ULL*24ULL*60ULL*60ULL*1'000'000'000ULL);

}  // namespace constant
}  // namespace gnc

#endif


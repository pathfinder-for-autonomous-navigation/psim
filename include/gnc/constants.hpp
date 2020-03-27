/** @file gnc/constants.hpp
 *  Autocoded constants header file. See tools/constants_generator.py for more
 *  information. */

#ifndef GNC_CONSTANTS_HPP_
#define GNC_CONSTANTS_HPP_

#include "config.hpp"

#include <lin/core/matrix/matrix.hpp>
#include <lin/core/vector/vector.hpp>

namespace gnc {
namespace constant {

GNC_TRACKED_CONSTANT(constexpr static double, pi, 3.141592653589793);

GNC_TRACKED_CONSTANT(constexpr static double, two_pi, 2.0 * pi);

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

GNC_TRACKED_CONSTANT(constexpr static double, b_noise_floor, 0.0);

GNC_TRACKED_CONSTANT(constexpr static double, max_mtr_moment, 0.113337 / 2.0);

GNC_TRACKED_CONSTANT(constexpr static lin::Matrix3x3f, JB_single_sat, 0.03798, 0.0, 0.0, 0.0, 0.03957, 0.0, 0.0, 0.0, 0.00688);

GNC_TRACKED_CONSTANT(constexpr static lin::Matrix3x3f, JB_docked_sats, 0.03798, 0.0, 0.0, 0.0, 0.03957, 0.0, 0.0, 0.0, 0.00688);

}  // namespace constant
}  // namespace gnc

#endif


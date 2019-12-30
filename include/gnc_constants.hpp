//
// include/gnc_constants.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/* - NOTICE -
 * 
 * Talk to Kyle Krol and/or Nathan Zimmerberg before editing this file.
 *
 * Values in this file should be seldomly edited and have hidden
 * interdependancies. The interdependancies and explicit calculation of the
 * below parameters are specified in MATLAB/config.m. */

#ifndef PAN_PSIM_INCLUDE_GNC_CONSTANTS_HPP_
#define PAN_PSIM_INCLUDE_GNC_CONSTANTS_HPP_

#include "lin.hpp"

namespace gnc {
namespace constant {

/** Good old pi. */
constexpr static double pi = 3.141592653589793L;

/** Good old pi (float version). */
constexpr static float pi_f = static_cast<float>(pi);

/** Two pi. */
constexpr static double two_pi = 2.0L * pi;

/** Two pi (float version). */
constexpr static float two_pi_f = static_cast<float>(two_pi);

/** Initial GPS week number that serves as the 'PAN epoch'. */
constexpr static unsigned short init_gps_week_number = 2045;

/** Initial GPS time of week for the 'PAN epoch'. This will normally be zero. */
constexpr static unsigned int init_gps_time_of_week = 0;

/** Initial GPS nanosecond count for the 'PAN epoch'. This will normally be
 *  zero. */
constexpr static unsigned long init_gps_nanoseconds = 0;

/** Initial decimal year that coincides with the 'PAN epoch'. */
constexpr static double init_dec_year = 2.019205478881278e3;

/** Initial decimal year that coincides with the 'PAN epoch' (float version). */
constexpr static float init_dec_year_f = static_cast<float>(init_dec_year);

/** Earth's intertial rotation rate about the z axis in the ECEF frame. */
constexpr static double earth_rate_ecef_z = 1.0e-04L * 0.729211585530000L;

/** Earth's intertial rotation rate about the z axis in the ECEF frame (float
 *  version). */
constexpr static float earth_rate_ecef_z_f = static_cast<float>(earth_rate_ecef_z);

/** Approximate precssion rate of Earth's Axis in radians per second. */
constexpr static lin::Vector3d earth_precession_rate = 1.0e-11L * lin::Vector3d({
  0.069896936522494L,
  -0.315732660584366L,
  0.0L
});

/** Approximate precssion rate of Earth's Axis in radians per second (float
 *  version). */
constexpr static lin::Vector3f earth_precession_rate_f = {
  static_cast<float>(earth_precession_rate(0)),
  static_cast<float>(earth_precession_rate(1)),
  static_cast<float>(earth_precession_rate(2))
};

/** Quaternion to rotate vectors in ECI to ECEF0. ECEF0 is defined to be the
 *  ECEF frame at time zero intertially stuck. */
constexpr static lin::Vector4d q_ecef0_eci = {
  0.000917219458782L,
  0.000038843277811L,
  0.998612074831165L,
  0.052660053181324L
};

/** Quaternion to rotate vectors in ECI to ECEF0. ECEF0 is defined to be the
 *  ECEF frame at time zero intertially stuck (float version). */
constexpr static lin::Vector4f q_ecef0_eci_f = {
  static_cast<float>(q_ecef0_eci(0)),
  static_cast<float>(q_ecef0_eci(1)),
  static_cast<float>(q_ecef0_eci(2)),
  static_cast<float>(q_ecef0_eci(3))
};

/** Eccentricity of Earth's orbit. */
constexpr static double earth_eccentricity = 0.0167086L;

/** Eccentricity of Earth's orbit (float version). */
constexpr static float earth_eccentricity_f = static_cast<float>(earth_eccentricity);

/** Time in seconds, relative to the PAN epoch, when Earth was at it's
 *  parihelion. */
constexpr static double earth_perihelion_time = -6287982.0L;

/** Time in seconds, relative to the PAN epoch, when Earth was at it's
 *  parihelion (float version). */
constexpr static float earth_perihelion_time_f = static_cast<float>(earth_perihelion_time);

/** Earth's orbital period in seconds. */
constexpr static double earth_period = 365.256363004L * 24.0L * 60.0L * 60.0L;

/** Earth's orbital period in seconds (float version). */
constexpr static float earth_period_f = static_cast<float>(earth_period);

/** Quaternion giving the rotation from Earth's perifocal frame to ECI. */
constexpr static lin::Vector4d q_eci_perifocal = {
   0.127456632677880L,
  -0.158124280715206L,
   0.762378784011859L,
  -0.614434787689722L
};

/** Quaternion giving the rotation from Earth's perifocal frame to ECI (float
 *  version). */
constexpr static lin::Vector4f q_eci_perifocal_f = {
  static_cast<float>(q_eci_perifocal(0)),
  static_cast<float>(q_eci_perifocal(1)),
  static_cast<float>(q_eci_perifocal(2)),
  static_cast<float>(q_eci_perifocal(3))
};

/** Noise floor of the magnetometer in units of T. */
constexpr static double b_noise_floor = 0.0L;

/** Noise floor of the magnetometer in units of T (float version). */
constexpr static float b_noise_floor_f = static_cast<float>(b_noise_floor);

/** Largest magnetic moment along a single axis that can be commanded of the
 *  ADCS system in units of Am^2. */
constexpr static double max_mtr_moment = 0.113337L / 2.0L;  // For one MTR

/** Largest magnetic moment along a single axis that can be commanded of the
 *  ADCS system in units of Am^2 (float version). */
constexpr static float max_mtr_moment_f = static_cast<float>(max_mtr_moment);
    
/** Dry moment of inertia of the satellite in body frame (kg*m^2)*/
constexpr static lin::Matrix3x3f JB_single_sat({
        0.0333f, 0.0f, 0.0f,
        0.0f, 0.0333f, 0.0f,
        0.0f, 0.0f, 0.0067f
    });
    
/** Dry moment of inertia of both satellites docked, in the body frame (kg*m^2)*/
constexpr static lin::Matrix3x3f JB_docked_sats({
        0.0333f, 0.0f, 0.0f,
        0.0f, 0.0333f, 0.0f,
        0.0f, 0.0f, 0.0067f
    });

}  // namespace constants
}  // namespace gnc

#endif

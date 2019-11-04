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
constexpr static double pi = 3.141592653589793;

/** Two pi. */
constexpr static double two_pi = 2.0 * pi;

/** Initial GPS week number that serves as the 'PAN epoch'. */
constexpr static unsigned short init_gps_week_number = 2045;

/** Initial GPS time of week for the 'PAN epoch'. This will normally be zero. */
constexpr static unsigned int init_gps_time_of_week = 0;

/** Initial GPS nanosecond count for the 'PAN epoch'. This will normally be
 *  zero. */
constexpr static unsigned long init_gps_nanoseconds = 0;

/** Initial decimal year that coincides with the 'PAN epoch'. */
constexpr static double init_dec_year = 2.019205478881278e3;

/** Earth's intertial rotation rate about the z axis in the ECEF frame. */
constexpr static double earth_rate_ecef_z = 1.0e-04 * 0.729211585530000;

/** Approximate precssion rate of Earth's Axis in radians per second. */
constexpr static lin::Vector3d earth_precession_rate = 1.0e-11 * lin::Vector3d({
  0.069896936522494,
  -0.315732660584366,
  0.0
});

/** Quaternion to rotate vectors in ECI to ECEF0. ECEF0 is defined to be the
 *  ECEF frame at time zero intertially stuck. */
constexpr static lin::Vector4d q_ecef0_eci = {
  0.000917219458782,
  0.000038843277811,
  0.998612074831165,
  0.052660053181324
};

/** Eccentricity of Earth's orbit. */
constexpr static double earth_eccentricity = 0.0167086;

/** Time in seconds, relative to the PAN epoch, when Earth was at it's
 *  parihelion. */
constexpr static double earth_perihelion_time = -6287982.0;

/** Earth's orbital period in seconds. */
constexpr static double earth_period = 365.256363004 * 24.0 * 60.0 * 60.0;

/** Quaternion giving the rotation from Earth's perifocal frame to ECI. */
constexpr static lin::Vector4d q_eci_perifocal = {
   0.127456632677880,
  -0.158124280715206,
   0.762378784011859,
  -0.614434787689722
};
}  // namespace constants
}  // namespace gnc

#endif

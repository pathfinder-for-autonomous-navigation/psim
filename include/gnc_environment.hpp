//
// include/gnc_environment.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_ENVIRONMENT_HPP_
#define PAN_PSIM_INCLUDE_GNC_ENVIRONMENT_HPP_

#include "lin.hpp"

namespace gnc {
namespace env {

/** @fn earth_attitude
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] q Quaternion to transform vectors from ECI to ECEF. */
void earth_attitude(double t, lin::Vector4d &q);

/** @fn earth_attitude
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] q Quaternion to transform vectors from ECI to ECEF. */
void earth_attitude(double t, lin::Vector4f &q);

/** @fn earth_angular_rate
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] w Angular rate to transform vectors from ECI to ECEF. */
void earth_angular_rate(double t, lin::Vector3d &w);

/** @fn earth_angular_rate
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] w Angular rate to transform vectors from ECI to ECEF. */
void earth_angular_rate(double t, lin::Vector3f &w);

/** @fn sun_vector
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] s Normalized vector from Earth the Sun in the ECI frame. */
void sun_vector(double t, lin::Vector3f &s);

/** @fn magnetic_field
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[in]  r Position in the ECEF frame with units of meters.
 *  @param[out] b Magnetic field in the ECEF frame with units of Tesla. */
void magnetic_field(double t, lin::Vector3f const &r, lin::Vector3f &b);

}  // namespace env
}  // namespace gnc

#endif

/** @file gnc/environment.hpp
 *  @author Kyle Krol */

#ifndef GNC_ENVIRONMENT_HPP_
#define GNC_ENVIRONMENT_HPP_

#include <lin/core.hpp>

namespace gnc {
namespace env {

/** @fn earth_attitude
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] q Quaternion to transform vectors from ECI to ECEF.
 *  @{ */
void earth_attitude(double t, lin::Vector4d &q);
void earth_attitude(double t, lin::Vector4f &q);
/** @} */

/** @fn earth_angular_rate
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] w Angular rate to transform vectors from ECI to ECEF.
 *  @{ */
void earth_angular_rate(double t, lin::Vector3d &w);
void earth_angular_rate(double t, lin::Vector3f &w);
/** @} */

/** @fn sun_vector
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[out] s Normalized vector from Earth the Sun in the ECI frame.
 *  @{ */
void sun_vector(double t, lin::Vector3f &s);
void sun_vector(double t, lin::Vector3d &s);
/** @} */

/** @fn magnetic_field
 *  @param[in]  t Time in seconds since the PAN epoch.
 *  @param[in]  r Position in the ECEF frame with units of meters.
 *  @param[out] b Magnetic field in the ECEF frame with units of Tesla.
 *  @{ */
void magnetic_field(double t, lin::Vector3d const &r, lin::Vector3f &b);
void magnetic_field(double t, lin::Vector3d const &r, lin::Vector3d &b);
/** @} */

}  // namespace env
}  // namespace gnc

#endif

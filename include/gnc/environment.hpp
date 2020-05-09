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

/** @file gnc/environment.hpp
 *  @author Kyle Krol
 */

/** @defgroup environment Environment
 *  @brief Contains environmental modules used throughout the rest of GNC.
 * 
 *  More details to come...
 */

#ifndef GNC_ENVIRONMENT_HPP_
#define GNC_ENVIRONMENT_HPP_

#include <lin/core.hpp>

namespace gnc {
namespace env {

/** @brief Determines earth's current attitude represented as a quaternion.
 *
 *  @param[in]  t          Time in seconds since the PAN epoch.
 *  @param[out] q_ecef_eci Earth's attitude represented as a quaternion.
 * 
 *  The returned quaternion represents the rotation transforming from ECI to ECEF.
 *  This representation will be most accurate around the PAN epoch.
 * 
 *  @sa env::earth_attitude(double, lin::Vector4d &)
 * 
 *  @ingroup environment
 */
void earth_attitude(double t, lin::Vector4d &q_ecef_eci);

/** @brief Determines earth's current attitude represented as a quaternion.
 *
 *  @param[in]  t          Time in seconds since the PAN epoch.
 *  @param[out] q_ecef_eci Earth's attitude represented as a quaternion.
 * 
 *  The returned quaternion represents the rotation transforming from ECI to ECEF.
 *  This representation will be most accurate around the PAN epoch.
 * 
 *  @sa env::earth_attitude(double, lin::Vector4f &)
 * 
 *  @ingroup environment
 */
void earth_attitude(double t, lin::Vector4f &q_ecef_eci);

/** @brief Determines Earth's angular rate.
 *
 *  @param[in]  t     Time in seconds since the PAN epoch.
 *  @param[out] w_eci Earth's angular rate (units of radians per second).
 *
 *  @internal
 *  Currently, the time input is extraneous as this function only looks at Earth's
 *  rate about the z-axis and doesn't model precession/nutation.
 *  @endinternal
 *
 *  @sa env::earth_angular_rate(double, lin::Vector3f)
 *
 *  @ingroup environment
 */
void earth_angular_rate(double t, lin::Vector3d &w_eci);

/** @brief Determines Earth's angular rate.
 *
 *  @param[in]  t     Time in seconds since the PAN epoch.
 *  @param[out] w_eci Earth's angular rate (units of radians per second).
 *
 *  @internal
 *  Currently, the time input is extraneous as this function only looks at Earth's
 *  rate about the z-axis and doesn't model precession/nutation.
 *  @endinternal
 *
 *  @sa env::earth_angular_rate(double, lin::Vector3d)
 *
 *  @ingroup environment
 */
void earth_angular_rate(double t, lin::Vector3f &w_eci);

/** @brief Models gravitational acceleration and potential due to Earth.
 * 
 *  @param[in]  r_ecef Position (units of meters)
 *  @param[out] g      Gravitational acceleration in ECEF (units of m/s^2).
 *  @param[out] U      Gravitational potential (units of J/kg).
 * 
 *  The calculation is done using a fourth order spherical harmonic expansion of
 *  Earth's gravitational potential.
 * 
 *  @sa env::gravity(lin::Vector3d const &, lin::Vector3d &, double &)
 * 
 *  @ingroup environment
 */
void gravity(lin::Vector3d const &r_ecef, lin::Vector3f &g_ecef, float &U_ecef);

/** @brief Models gravitational acceleration and potential due to Earth.
 * 
 *  @param[in]  r_ecef Position (units of meters)
 *  @param[out] g_ecef Gravitational acceleration (units of m/s^2).
 *  @param[out] U_ecef Gravitational potential (units of J/kg).
 * 
 *  The calculation is done using a fourth order spherical harmonic expansion of
 *  Earth's gravitational potential.
 * 
 *  @sa env::gravity(lin::Vector3d const &, lin::Vector3f &, float &)
 * 
 *  @ingroup environment
 */
void gravity(lin::Vector3d const &r_ecef, lin::Vector3d &g_ecef, double &U_ecef);

/** @brief Predicts the vector pointing from Earth to the sun.
 * 
 *  @param[in]  t     Time in seconds since the PAN epoch.
 *  @param[out] s_eci Vector pointing from Earth to the sun in ECI (unit vector).
 * 
 *  This vector is commonly taken as the "sat to sun" vector throughout GNC code.
 * 
 *  @sa env::sun_vector(double, lin::Vector3d &)
 * 
 *  @ingroup environment
 */
void sun_vector(double t, lin::Vector3f &s_eci);

/** @brief Predicts the vector pointing from Earth to the sun.
 * 
 *  @param[in]  t     Time in seconds since the PAN epoch.
 *  @param[out] s_eci Vector pointing from Earth to the sun in ECI (unit vector).
 * 
 *  This vector is commonly taken as the "sat to sun" vector throughout GNC code.
 * 
 *  @sa env::sun_vector(double, lin::Vector3f &)
 * 
 *  @ingroup environment
 */
void sun_vector(double t, lin::Vector3d &s_eci);

/** @brief Models Earth's magnetic field as a function of time and position.
 * 
 *  @param[in] t      Time in seconds since the PAN epoch.
 *  @param[in] r_ecef Position (units of meters).
 *  @param[in] b_ecef Magnetic field (units of Tesla).
 * 
 *  @sa env::magnetic_field(double, lin::Vector3d const &, lin::Vector3d &)
 * 
 *  @ingroup environment
 */
void magnetic_field(double t, lin::Vector3d const &r_ecef, lin::Vector3f &b_ecef);

/** @brief Models Earth's magnetic field as a function of time and position.
 * 
 *  @param[in] t      Time in seconds since the PAN epoch.
 *  @param[in] r_ecef Position (units of meters).
 *  @param[in] b_ecef Magnetic field (units of Tesla).
 * 
 *  @sa env::magnetic_field(double, lin::Vector3d const &, lin::Vector3f &)
 * 
 *  @ingroup environment
 */
void magnetic_field(double t, lin::Vector3d const &r_ecef, lin::Vector3d &b_ecef);

}  // namespace env
}  // namespace gnc

#endif

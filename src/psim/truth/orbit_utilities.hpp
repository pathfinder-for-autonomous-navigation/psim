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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/truth/orbit_utilities.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_ORBIT_UTILITIES_HPP_
#define PSIM_TRUTH_ORBIT_UTILITIES_HPP_

#include <psim/core/types.hpp>

namespace psim {
namespace orbit {

/** @brief Calculate gravitational acceleration.
 *
 *  @param[in]  r_ecef Position in ECEF (m).
 *  @param[out] g_ecef Gravitational accelerating in ECEF (m/s^2).
 */
void gravity(Vector3 const &r_ecef, Vector3 &g_ecef);

/** @brief Calculate gravitational acceleration and potential.
 *
 *  @param[in]  r_ecef Position in ECEF (m).
 *  @param[out] g_ecef Gravitational accelerating in ECEF (m/s^2).
 *  @param[out] U      Gravitational potential (J/kg).
 */
void gravity(Vector3 const &r_ecef, Vector3 &g_ecef, Real &U);

/** @brief Calculate atmospheric density.
 *
 *  @param[in]  r_ecef Position in ECEF (m).
 *  @param[out] rho    Atmospheric density (k/m^3);
 */
void density(Vector3 const &r_ecef, Real &rho);

/** @brief Calculate the acceleration due to drag.
 *
 *  @param[in]  r_ecef Position in ECEF (m).
 *  @param[in]  v_ecef Velocity in ECEF (m/s).
 *  @param[in]  S      Area projected along the direction of travel (m^2).
 *  @param[in]  m      Satellite mass (kg).
 *  @param[out] a_ecef Drag acceleration in ECEF (m/s^2).
 */
void drag(Vector3 const &r_ecef, Vector3 const &v_ecef, Real S, Real m,
    Vector3 &a_ecef);

/** @brief Calculates total orbital acceleration in ECEF.
 *
 *  @param[in] earth_w     Earth's angular rate in ECEF (rad/s).
 *  @param[in] earth_w_dot Time derivative of Earth's angular rate in ECEF
 *                         (rad/s^2)
 *  @param[in] r_ecef      Position in ECEF (m).
 *  @param[in] v_ecef      Velocity in ECEF (m/s).
 *  @param[in] S           Area projected along the direction of travel (m^2).
 *  @param[in] m           Satellite mass (kg).
 */
void acceleration(Vector3 const &earth_w, Vector3 const &earth_w_dot,
    Vector3 const &r_ecef, Vector3 const &v_ecef, Real S, Real m,
    Vector3 a_ecef);

} // namespace orbit
} // namespace psim

#endif

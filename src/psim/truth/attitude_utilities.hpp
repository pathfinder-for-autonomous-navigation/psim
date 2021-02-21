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

/** @file psim/truth/attitude_utilities.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_ATTITUDE_UTILITIES_HPP_
#define PSIM_TRUTH_ATTITUDE_UTILITIES_HPP_

#include <psim/core/types.hpp>

namespace psim {
namespace attitude {

/** @brief Calculates satellite surface area projected along the direction of
 *         travel.
 *
 *  @param[in] q_body_eci Rotation from ECI to body.
 *  @param[in] q_eci_ecef Rotation from ECEF to ECI.
 *  @param[in] v_ecef     Velocity in ECEC (m/s).
 *
 *  @return Surface area projected along the direction of travel (m^2).
 */
Real S(Vector4 const &q_body_eci, Vector4 const &q_eci_ecef,
    Vector3 const &v_ecef);

} // namespace attitude
} // namespace psim

#endif

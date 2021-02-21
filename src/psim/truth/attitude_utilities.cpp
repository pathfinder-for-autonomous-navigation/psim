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

/** @file psim/truth/attitude_utilities.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/attitude_utilities.hpp>

#include <gnc/config.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/math.hpp>

namespace psim {
namespace attitude {

Real S(Vector4 const &q_body_eci, Vector4 const &q_eci_ecef,
    Vector3 const &v_ecef) {
  static constexpr Vector3 PSIM_AREA_VEC = {0.03, 0.03, 0.01};

  Vector3 v_body;
  gnc::utl::rotate_frame(q_eci_ecef, v_ecef, v_body);
  gnc::utl::rotate_frame(q_body_eci, v_body);

  return lin::dot(lin::abs(v_body / lin::norm(v_body)), PSIM_AREA_VEC);
}
} // namespace attitude
} // namespace psim

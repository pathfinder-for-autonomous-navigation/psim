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

/** @file psim/truth/transform_direction.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/transform_direction.hpp>

#include <gnc/utilities.hpp>

namespace psim {

Vector3 TransformDirectionBody::vector_body() const {
  return vector->get();
}

Vector3 TransformDirectionBody::vector_ecef() const {
  auto const &d_eci = Super::vector_eci.get();
  auto const &q_ecef_eci = truth_earth_q_ecef_eci->get();

  Vector3 d_ecef;
  gnc::utl::rotate_frame(q_ecef_eci, d_eci, d_ecef);
  return d_ecef;
}

Vector3 TransformDirectionBody::vector_eci() const {
  auto const &d_body = vector->get();
  auto const &q_eci_body = truth_satellite_attitude_q_eci_body->get();

  Vector3 d_eci;
  gnc::utl::rotate_frame(q_eci_body, d_body, d_eci);
  return d_eci;
}

Vector3 TransformDirectionEcef::vector_body() const {
  auto const &d_eci = Super::vector_eci.get();
  auto const q_body_eci = truth_satellite_attitude_q_body_eci->get();

  Vector3 d_body;
  gnc::utl::rotate_frame(q_body_eci, d_eci, d_body);
  return d_body;
}

Vector3 TransformDirectionEcef::vector_ecef() const {
  return vector->get();
}

Vector3 TransformDirectionEcef::vector_eci() const {
  auto const &d_ecef = vector->get();
  auto const &q_eci_ecef = truth_earth_q_eci_ecef->get();

  Vector3 d_eci;
  gnc::utl::rotate_frame(q_eci_ecef, d_ecef, d_eci);
  return d_eci;
}

Vector3 TransformDirectionEci::vector_body() const {
  auto const &d_eci = vector->get();
  auto const &q_body_eci = truth_satellite_attitude_q_body_eci->get();

  Vector3 d_body;
  gnc::utl::rotate_frame(q_body_eci, d_eci, d_body);
  return d_body;
}

Vector3 TransformDirectionEci::vector_ecef() const {
  auto const &d_eci = vector->get();
  auto const &q_ecef_eci = truth_earth_q_ecef_eci->get();

  Vector3 d_ecef;
  gnc::utl::rotate_frame(q_ecef_eci, d_eci, d_ecef);
  return d_ecef;
}

Vector3 TransformDirectionEci::vector_eci() const {
  return vector->get();
}
}  // namespace psim

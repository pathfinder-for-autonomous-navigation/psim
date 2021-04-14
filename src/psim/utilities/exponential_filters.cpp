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

/** @file psim/utilities/exponential_filters.cpp
 *  @author Kyle Krol
 */

#include <psim/utilities/exponential_filters.hpp>

#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace psim {

template <typename T>
static inline constexpr T filter(T const &x, Real alpha, T const &z) {
  constexpr Real one = 1.0;

  if (lin::all(lin::isfinite(x))) {
    return alpha * z + (one - alpha) * x;
  } else {
    return z;
  }
}

void ExponentialFilterReal::add_fields(State &state) {
  this->Super::add_fields(state);

  real_filtered.get() = gnc::constant::nan;
}

void ExponentialFilterReal::step() {
  auto const &z = real->get();
  auto const &a = real_alpha.get();
  auto &x = real_filtered.get();

  x = filter(x, a, z);
}

void ExponentialFilterVector2::add_fields(State &state) {
  this->Super::add_fields(state);

  vector_filtered.get() = lin::nans<Vector2>();
}

void ExponentialFilterVector2::step() {
  auto const &z = vector->get();
  auto const &a = vector_alpha.get();
  auto &x = vector_filtered.get();

  x = filter(x, a, z);
}

void ExponentialFilterVector3::add_fields(State &state) {
  this->Super::add_fields(state);

  vector_filtered.get() = lin::nans<Vector3>();
}

void ExponentialFilterVector3::step() {
  auto const &z = vector->get();
  auto const &a = vector_alpha.get();
  auto &x = vector_filtered.get();

  x = filter(x, a, z);
}

void ExponentialFilterVector4::add_fields(State &state) {
  this->Super::add_fields(state);

  vector_filtered.get() = lin::nans<Vector4>();
}

void ExponentialFilterVector4::step() {
  auto const &z = vector->get();
  auto const &a = vector_alpha.get();
  auto &x = vector_filtered.get();

  x = filter(x, a, z);
}
} // namespace psim

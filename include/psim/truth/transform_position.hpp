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

/** @file psim/truth/transform_position.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_TRANSFORM_POSITION_HPP_
#define PSIM_TRUTH_TRANSFORM_POSITION_HPP_

#include <psim/truth/transform_position.yml.hpp>

namespace psim {

/** @brief Position transformer taking a position in ECEF as input.
 */
class TransformPositionEcef : public TransformPosition<TransformPositionEcef> {
 private:
  typedef TransformPosition<TransformPositionEcef> Super;

 public:
  using Super::TransformPosition;

  TransformPositionEcef() = delete;
  virtual ~TransformPositionEcef() = default;

  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};

/** @brief Position transformer taking a position in ECI as input.
 */
class TransformPositionEci : public TransformPosition<TransformPositionEci> {
 private:
  typedef TransformPosition<TransformPositionEci> Super;

 public:
  using Super::TransformPosition;

  TransformPositionEci() = delete;
  virtual ~TransformPositionEci() = default;

  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};
}  // namespace psim

#endif

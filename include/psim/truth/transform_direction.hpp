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

/** @file psim/truth/transform_direction.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_TRANSFORM_DIRECTION_HPP_
#define PSIM_TRUTH_TRANSFORM_DIRECTION_HPP_

#include <psim/truth/transform_direction.yml.hpp>

namespace psim {

/** @brief Direction transformer taking input in the body frame.
 */
class TransformDirectionBody : public TransformDirection<TransformDirectionBody> {
 private:
  typedef TransformDirection<TransformDirectionBody> Super;

 public:
  using Super::TransformDirection;

  TransformDirectionBody() = delete;
  virtual ~TransformDirectionBody() = default;

  Vector3 vector_body() const;
  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};

/** @brief Direction transformer taking input in ECEF.
 */
class TransformDirectionEcef : public TransformDirection<TransformDirectionEcef> {
 private:
  typedef TransformDirection<TransformDirectionEcef> Super;

 public:
  using Super::TransformDirection;

  TransformDirectionEcef() = delete;
  virtual ~TransformDirectionEcef() = default;

  Vector3 vector_body() const;
  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};

/** @brief Direction transformer taking input in ECI.
 */
class TransformDirectionEci : public TransformDirection<TransformDirectionEci> {
 private:
  typedef TransformDirection<TransformDirectionEci> Super;

 public:
  using Super::TransformDirection;

  TransformDirectionEci() = delete;
  virtual ~TransformDirectionEci() = default;

  Vector3 vector_body() const;
  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};
}  // namespace psim

#endif

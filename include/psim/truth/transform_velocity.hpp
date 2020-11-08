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

#ifndef PSIM_TRUTH_TRANSFORM_VELOCITY_HPP_
#define PSIM_TRUTH_TRANSFORM_VELOCITY_HPP_

#include <psim/truth/transform_velocity.yml.hpp>

namespace psim {

/** @brief Velocity transformer taking a velocity in ECEF as input.
 */
class TransformVelocityEcef : public TransformVelocity<TransformVelocityEcef> {
 private:
  typedef TransformVelocity<TransformVelocityEcef> Super;

 public:
  TransformVelocityEcef() = delete;
  virtual ~TransformVelocityEcef() = default;

  /** @brief Sets the frame arguments to ECEF.
   */
  TransformVelocityEcef(Configuration const &config,
      std::string const &satellite, std::string const &vector);

  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};

/** @brief Velocity transformer taking a velocity in ECI as input.
 */
class TransformVelocityEci : public TransformVelocity<TransformVelocityEci> {
 private:
  typedef TransformVelocity<TransformVelocityEci> Super;

 public:
  TransformVelocityEci() = delete;
  virtual ~TransformVelocityEci() = default;

  /** @brief Sets the frame arguments to ECI.
   */
  TransformVelocityEci(Configuration const &config,
      std::string const &satellite, std::string const &vector);

  Vector3 vector_ecef() const;
  Vector3 vector_eci() const;
};
}  // namespace psim

#endif

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

/** @file gnc/attitude_estimate.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_ATTITUDE_ESTIMATE_HPP_
#define GNC_ATTITUDE_ESTIMATE_HPP_

#include <gnc/config.hpp>

#include <lin/core.hpp>

namespace gnc {

/** @brief
 */
class AttitudeEstimate {
 public:
  /** @brief Type representing real scalars within the class.
   */
  typedef double Real;

  /** @brief Type representing time in nanoseconds within the class.
   */
  typedef int32_t Time;

  /** @brief Convenience template for defining lin matrix types within the
   *         class.
   */
  template <lin::size_t R, lin::size_t C, lin::size_t MR = R, lin::size_t MC = C>
  using Matrix = lin::Matrix<Real, R, C, MR, MC>;

  /** @brief Convenience template for defining lin row vector types within the
   *         class.
   */
  template <lin::size_t N, lin::size_t MN = N>
  using RowVector = lin::RowVector<Real, N, MN>;

  /** @brief Convenience template for defining lin vector types within the
   *         class.
   */
  template <lin::size_t N, lin::size_t MN = N>
  using Vector = lin::Vector<Real, N, MN>;

 private:
  /** @internal
   *
   *  @brief
   */
  Vector<6> _x;

  /** @internal
   *
   *  @brief
   */
  Vector<6> _xs[12]

  /** @internal
   * 
   *  @brief
   */
  Matrix<6, 6> _sqrtP;
};
}  // namespace gnc

#endif

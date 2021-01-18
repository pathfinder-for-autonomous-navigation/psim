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

/** @file gnc/orbit_estimator.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_ORBIT_ESTIMATOR_HPP_
#define GNC_ORBIT_ESTIMATOR_HPP_

#include <gnc/orbit.hpp>
#include <lin/core.hpp>

#include <cstdint>

namespace gnc {

class OrbitEstimator : public Orbit {
 protected:
  lin::Matrixd<6, 6> _P;

  void _check_validity();
  void _reset(double dt, lin::Vectord<3> const &w, lin::Vectord<3> const &dz,
      lin::Vectord<3> const &r);
  void _update(double dt, lin::Vectord<3> const &w, lin::Vectord<3> const &z,
      lin::Vectord<3> const &q, lin::Vectord<3> const &r);

 public:
  OrbitEstimator() = default;
  OrbitEstimator(OrbitEstimator const &) = default;
  OrbitEstimator(OrbitEstimator &&) = default;
  OrbitEstimator &operator=(OrbitEstimator const &) = default;
  OrbitEstimator &operator=(OrbitEstimator &&) = default;

  void reset(
      int64_t dt_ns, lin::Vectord<3> const &dr_ecef, lin::Vectord<3> const &r);

  void update(int64_t dt_ns, lin::Vectord<3> const &w_ecef,
      lin::Vectord<3> r_ecef, lin::Vectord<3> const &q,
      lin::Vectord<3> const &r);
};
} // namespace gnc

#endif

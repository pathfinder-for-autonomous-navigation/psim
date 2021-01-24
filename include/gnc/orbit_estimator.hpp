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

#include "orbit.hpp"

namespace gnc {

class OrbitEstimator : public Orbit {
 protected:
  Matrix<6, 6> _S;

  void _check_validity();
  void _dv(Vector<3> const &dv, Vector<3> const &ds);
  void _reset(Vector<3> const &w, Vector<3> const &r, Vector<3> const &v,
      Vector<6> const &s);
  void _update(Real dt, Vector<3> const &w, Vector<6> const &sqrt_q);
  void _update(Real dt, Vector<3> const &w, Vector<3> const &r,
      Vector<3> const &v, Vector<6> const &sqrt_q, Vector<6> const &sqrt_r);

 public:
  OrbitEstimator() = default;
  OrbitEstimator(OrbitEstimator const &) = default;
  OrbitEstimator(OrbitEstimator &&) = default;
  OrbitEstimator &operator=(OrbitEstimator const &) = default;
  OrbitEstimator &operator=(OrbitEstimator &&) = default;

  OrbitEstimator(Vector<3> const &w_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef, Vector<6> const &s);

  Vector<3> r_ecef_sigma() const;
  Vector<3> v_ecef_sigma() const;

  void dv(Vector<3> const &dv_ecef, Vector<3> const &ds);
  void reset(Vector<3> const &w_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef, Vector<6> const &s);
  void update(Time dt_ns, Vector<3> const &w_ecef, Vector<6> const &sqrt_q);
  void update(Time dt_ns, Vector<3> const &w_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef, Vector<6> const &sqrt_q, Vector<6> const &sqrt_r);
};
} // namespace gnc

#endif

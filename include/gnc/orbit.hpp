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

/** @file include/orbit.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_ORBIT_HPP_
#define GNC_ORBIT_HPP_

#include "config.hpp"
#include "constants.hpp"

#include <gnc/ode4.hpp>
#include <lin/core.hpp>

#include <cstdint>

namespace gnc {

class Orbit {
 protected:
  typedef double Real;
  typedef int64_t Time;
  template <lin::size_t N, lin::size_t MN = N>
  using Vector = lin::Vector<Real, N, MN>;
  template <lin::size_t N, lin::size_t MN = N>
  using RowVector = lin::RowVector<Real, N, MN>;
  template <lin::size_t R, lin::size_t C, lin::size_t MR = R, lin::size_t MC = C>
  using Matrix = lin::Matrix<Real, R, C, MR, MC>;

  static constexpr Real mu = constant::mu_earth;

  inline static Real _time(Time t) { return static_cast<Real>(t) * 1.0e-9; }
  static void _dot(Real mu, Vector<3> const &w, Vector<6> const &x, Vector<6> &dx);
  static void _jac(Real mu, Vector<3> const &w, Vector<6> const &x, Matrix<6, 6> &J);

 protected:
  gnc::Ode4<Real, 6> _ode;
  bool _is_valid = false;
  Vector<3> _w;
  Vector<6> _x;

  void _check_validity();
  void _dv(Vector<3> const &dv);
  void _reset(Vector<3> const &w, Vector<3> const &r, Vector<3> const &v);
  void _update(Real dt, Vector<3> const &w);

 public:
  Orbit() = default;
  Orbit(Orbit const &) = default;
  Orbit(Orbit &&) = default;
  Orbit &operator=(Orbit const &) = default;
  Orbit &operator=(Orbit &&) = default;

  Orbit(Vector<3> const &w_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef);

  inline bool is_valid() const { return _is_valid; }
  Vector<3> r_ecef() const;
  Vector<3> v_ecef() const;

  void dv(Vector<3> const &dv_ecef);
  void reset(Vector<3> const &w_ecef, Vector<3> const &r_ecef, Vector<3> const &v_ecef);
  void update(Time dt_ns, Vector<3> const &w_ecef);
};
} // namespace gnc

#endif

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

#include <gnc/ode4.hpp>
#include <lin/core.hpp>

#include <cstdint>

namespace gnc {

class Orbit {
 protected:
  static void _dot(double mu, lin::Vectord<3> const &w,
      lin::Vectord<6> const &x, lin::Vectord<6> &dx);
  static void _jac(double mu, lin::Vectord<3> const &w,
      lin::Vectord<6> const &x, lin::Matrixd<6, 6> &J);

 private:
  gnc::Ode4<double, 6> _ode;

 protected:
  bool _is_valid = false;
  lin::Vectord<6> _x;

  void _check_validity();
  void _update(double dt, lin::Vectord<3> const &w_ecwef);

 public:
  Orbit() = default;
  Orbit(Orbit const &) = default;
  Orbit(Orbit &&) = default;
  Orbit &operator=(Orbit const &) = default;
  Orbit &operetor = (Orbit &&) = default;

  Orbit(lin::Vectord<3> const &r_ecef, lin::Vectord<3> const &v_ecef);

  bool is_valid() const;
  lin::Vectord<3> r_ecef() const;
  lin::Vectord<3> v_ecef() const;

  void dv(lin::Vectord<3> const &dv_ecef);
  void update(int64_t dt, lin::Vectord<3> const &w_ecef);
};
} // namespace gnc

#endif

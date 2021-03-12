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

/** @file psim/truth/orbit_utilities.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/orbit_utilities.hpp>

#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/math.hpp>

#include <GGM05S.hpp>
#include <geograv.hpp>

namespace psim {
namespace orbit {

Vector3 gravity(Vector3 const &r_ecef) {
  Real _;
  return gravity(r_ecef, _);
}

Vector3 gravity(Vector3 const &r_ecef, Real &U) {
  static constexpr auto order = 11;
  static constexpr auto grav = static_cast<geograv::Coeff<order>>(GGM05S);

  geograv::Vector g, r = {r_ecef(0), r_ecef(1), r_ecef(2)};
  U = geograv::GeoGrav(r, g, grav, true);
  return {g.x, g.y, g.z};
}

Real density(Vector3 const &r_ecef) {
  /* Atmospheric density model is pulled from section 11.2.1 of "Fundamentals of
   * Spacecraft Attitude Determination and Control" by Markley and Crassidis.
   */
  static constexpr lin::size_t I = 36;
  static constexpr Vector<I> h0 = {0.0, 25.0e3, 30.0e3, 35.0e3, 40.0e3, 45.0e3,
      50.0e3, 55.0e3, 60.0e3, 65.0e3, 70.0e3, 75.0e3, 80.0e3, 85.0e3, 90.0e3,
      95.0e3, 100.0e3, 110.0e3, 120.0e3, 130.0e3, 140.0e3, 150.0e3, 160.0e3,
      180.0e3, 200.0e3, 250.0e3, 300.0e3, 350.0e3, 400.0e3, 450.0e3, 500.0e3,
      600.0e3, 700.0e3, 800.0e3, 900.0e3, 1000.0e3};
  static constexpr Vector<I> p0 = {1.225, 3.899e-2, 1.774e-2, 8.279e-3,
      3.972e-3, 1.995e-3, 1.057e-3, 5.821e-4, 3.206e-4, 1.718e-4, 8.770e-5,
      4.178e-5, 1.905e-5, 8.337e-6, 3.396e-6, 1.343e-6, 5.297e-7, 9.661e-8,
      2.438e-8, 8.484e-9, 3.845e-9, 2.070e-9, 1.224e-9, 5.464e-10, 2.789e-10,
      7.248e-11, 2.418e-11, 9.158e-12, 3.725e-12, 1.585e-12, 6.967e-13,
      1.454e-13, 3.614e-14, 1.170e-14, 5.245e-15, 3.019e-15};
  static constexpr Vector<I> H = {8.44e3, 6.49e3, 6.75e3, 7.07e3, 7.47e3,
      7.83e3, 7.95e3, 7.73e3, 7.29e3, 6.81e3, 6.33e3, 6.00e3, 5.70e3, 5.41e3,
      5.38e3, 5.74e3, 6.15e3, 8.06e3, 11.6e3, 16.1e3, 20.6e3, 24.6e3, 26.3e3,
      33.2e3, 38.5e3, 46.9e3, 52.5e3, 56.4e3, 59.4e3, 62.2e3, 65.8e3, 79.0e3,
      109.0e3, 164.0e3, 225.0e3, 268.0e3};

  // Calculate our altitude
  auto const h = lin::norm(r_ecef) - gnc::constant::r_earth;

  // Determine the appropriate index
  lin::size_t i = h0.size() - 1;
  while (h < h0(i) && --i)
    ;

  // Atmospheric density calculation
  return p0(i) * lin::exp((h0(i) - h) / H(i));
}

Vector3 drag(Vector3 const &r_ecef, Vector3 const &v_ecef, Real S, Real m) {
  static constexpr Real half = 0.5;

  /* Drag coefficient pulled from "An Evaluation of CubeSat Orbital Decay" by
   * Oltrogge and Leveque.
   *
   * https://digitalcommons.usu.edu/cgi/viewcontent.cgi?article=1144&context=smallsat
   */
  static constexpr Real Cd = 2.2;

  /* Section 10.3.3 of "Fundamental of Spacecraft Attitude Determination and
   * Control" by Markley and Crassidis gives force due to atmospheric drag as:
   *
   *  F_drag = -1/2 rho Cd S |v| v
   *
   * where F_drag is the force of drag, rho is atmospheric density, Cd is a
   * dimensionaless drag coefficient, S is the object area projected along the
   * direction of travel, and v is the velocity relative to the atmosphere (i.e.
   * in ECEF because the atmosphere rotates).
   */
  auto const rho = density(r_ecef);
  return v_ecef * (-half * Cd * S * rho * lin::norm(v_ecef) / m);
}

Vector3 rotational(Vector3 const &earth_w, Vector3 const &earth_w_dot,
    Vector3 const &r_ecef, Vector3 const &v_ecef) {
  static constexpr Real two = 2.0;

  /* Acceleration due to the rotating frame.
   *
   * https://en.wikipedia.org/wiki/Rotating_reference_frame#Relation_between_accelerations_in_the_two_frames
   */
  return -(two * lin::cross(earth_w, v_ecef) +
           lin::cross(earth_w, lin::cross(earth_w, r_ecef)) +
           lin::cross(earth_w_dot, r_ecef));
}

Vector3 acceleration(Vector3 const &earth_w, Vector3 const &earth_w_dot,
    Vector3 const &r_ecef, Vector3 const &v_ecef, Real S, Real m) {
  /* Numerically, starting with the smaller forces first like fake forces and
   * drag will reduce rounding errors. Therefore, we included the force of
   * gravity last here.
   */
  auto const a_drag_ecef = drag(r_ecef, v_ecef, S, m);
  auto const a_grav_ecef = gravity(r_ecef);
  auto const a_rot_ecef = rotational(earth_w, earth_w_dot, r_ecef, v_ecef);

  return (a_rot_ecef + a_drag_ecef) + a_grav_ecef;
}
} // namespace orbit
} // namespace psim

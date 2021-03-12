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

/** @file psim/truth/hill_frame.cpp
 *  @author Kyle Krol
 */
#define LIN_DESKTOP
#include <lin/core.hpp>
#include <psim/truth/hill_frame.hpp>
#include <iostream>

#include <gnc/utilities.hpp>

namespace psim {

HillFrameEci::HillFrameEci(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite,
    std::string const &other)
  : Super(randoms, config, satellite, other, "eci") {}

Vector4 HillFrameEci::truth_satellite_hill_q_hill_frame() const {
  auto const &satellite_r = truth_satellite_orbit_r_frame->get();
  auto const &satellite_v = truth_satellite_orbit_v_frame->get();

  Matrix<3, 3> Q_hill_frame;
  gnc::utl::dcm(Q_hill_frame, satellite_r, satellite_v);

  Vector<4> q_hill_frame;
  gnc::utl::dcm_to_quat(Q_hill_frame, q_hill_frame);
  return q_hill_frame;
}

Vector3 HillFrameEci::truth_satellite_hill_w_frame() const {
  auto const &satellite_r = truth_satellite_orbit_r_frame->get();
  auto const &satellite_v = truth_satellite_orbit_v_frame->get();

  return lin::cross(satellite_r, satellite_v) / lin::fro(satellite_r);
}

Vector3 HillFrameEci::truth_satellite_hill_dr() const {
  auto const &q_hill_frame = Super::truth_satellite_hill_q_hill_frame.get();
  auto const &satellite_r = truth_satellite_orbit_r_frame->get();
  auto const &other_r = truth_other_orbit_r_frame->get();

  Vector3 dr = other_r - satellite_r;
  gnc::utl::rotate_frame(q_hill_frame, dr);
  return dr;
}

Vector3 HillFrameEci::truth_satellite_hill_dv() const {
  auto const &q_hill_frame = Super::truth_satellite_hill_q_hill_frame.get();
  auto const &w_hill_frame = Super::truth_satellite_hill_w_frame.get();
  auto const &dr = Super::truth_satellite_hill_dr.get();
  auto const &satellite_v = truth_satellite_orbit_v_frame->get();
  auto const &other_v = truth_other_orbit_v_frame->get();

  Vector3 dv = other_v - satellite_v;
  gnc::utl::rotate_frame(q_hill_frame, dv);
  return dv - lin::cross(Vector3({0.0, 0.0, lin::norm(w_hill_frame)}), dr);
}
} // namespace psim

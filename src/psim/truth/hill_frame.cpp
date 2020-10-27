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

#include <psim/truth/hill_frame.hpp>

#include <gnc/utilities.hpp>

#include <lin/core.hpp>

namespace psim {

HillFrameEci::HillFrameEci(Configuration const &config,
    std::string const &leader, std::string const &follower)
  : Super(config, leader, follower, "eci") { }

Vector4 HillFrameEci::prefix_leader_q_hill_frame() const {
  auto const &leader_r = this->prefix_leader_orbit_r_frame->get();
  auto const &leader_v = this->prefix_leader_orbit_r_frame->get();

  
}

Vector3 HillFrameEci::prefix_leader_w_hill() const {
  auto const &leader_r = this->prefix_leader_orbit_r_frame->get();
  auto const &leader_v = this->prefix_leader_orbit_r_frame->get();

  return lin::cross(leader_r, leader_v) / lin::fro(leader_v);
}

Vector3 HillFrameEci::prefix_follower_orbit_r_hill() const {

}

Vector3 HillFrameEci::prefix_follower_orbit_v_hill() const {

}
}  // namespace psim

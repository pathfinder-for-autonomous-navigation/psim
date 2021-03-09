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

/** @file psim/fc/orbit_controller.hpp
 * @author Govind Chari
 */

#ifndef PSIM_FC_ORBIT_CONTROLLER_HPP_
#define PSIM_FC_ORBIT_CONTROLLER_HPP_

#include <psim/fc/orbit_controller.yml.hpp>

#include <gnc/orbit_controller.hpp>

namespace psim {

class OrbitController : public OrbitControllerInterface<OrbitController> {
 private:
  typedef OrbitControllerInterface<OrbitController> Super;

  gnc::OrbitControllerState _orbitController;

  Integer last_firing = 0;

 public:
  using Super::OrbitControllerInterface;

  OrbitController() = delete;
  virtual ~OrbitController() = default;

  virtual void step() override;
};
} // namespace psim

#endif
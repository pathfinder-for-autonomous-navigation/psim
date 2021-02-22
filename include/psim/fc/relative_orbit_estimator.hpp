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

/** @file psim/fc/attitude_estimator.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_FC_RELATIVE_ORBIT_ESTIMATOR_HPP_
#define PSIM_FC_RELATIVE_ORBIT_ESTIMATOR_HPP_

#include <psim/fc/relative_orbit_estimator.yml.hpp>

#include <gnc/relative_orbit_estimate.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

class RelativeOrbitEstimator : public RelativeOrbitEstimatorInterface<RelativeOrbitEstimator> {
 private:
  typedef RelativeOrbitEstimatorInterface<RelativeOrbitEstimator> Super;

  Vector3 previous_dr = lin::nans<Vector3>();
  gnc::RelativeOrbitEstimate estimate;

  void _set_relative_orbit_outputs();

 public:
  using Super::RelativeOrbitEstimatorInterface;

  RelativeOrbitEstimator() = delete;
  virtual ~RelativeOrbitEstimator() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;

  Vector3 fc_satellite_relative_orbit_dr_error() const;
  Vector3 fc_satellite_relative_orbit_r_hill_error() const;
  Vector3 fc_satellite_relative_orbit_dv_error() const;
  Vector3 fc_satellite_relative_orbit_v_hill_error() const;
};
} // namespace psim

#endif

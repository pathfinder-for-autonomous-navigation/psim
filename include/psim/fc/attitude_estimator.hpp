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

#ifndef PSIM_FC_ATTITUDE_ESTIMATOR_HPP_
#define PSIM_FC_ATTITUDE_ESTIMATOR_HPP_

#include <psim/fc/attitude_estimator.yml.hpp>

#include <gnc/attitude_estimator.hpp>

namespace psim {

class AttitudeEstimator : public AttitudeEstimatorInterface<AttitudeEstimator> {
 private:
  typedef AttitudeEstimatorInterface<AttitudeEstimator> Super;

  gnc::AttitudeEstimatorState _attitude_state;
  gnc::AttitudeEstimatorData _attitude_data;
  gnc::AttitudeEstimate _attitude_estimate;

  void _set_attitude_outputs();

 public:
  using Super::AttitudeEstimatorInterface;

  AttitudeEstimator() = delete;
  virtual ~AttitudeEstimator() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;

  Vector4 fc_satellite_attitude_q_body_eci_error() const;
  Real fc_satellite_attitude_q_body_eci_error_degrees() const;
  Vector3 fc_satellite_attitude_p_body_eci_error() const;
  Vector3 fc_satellite_attitude_p_body_eci_sigma() const;
  Vector3 fc_satellite_attitude_w() const;
  Vector3 fc_satellite_attitude_w_error() const;
  Vector3 fc_satellite_attitude_w_bias_error() const;
  Vector3 fc_satellite_attitude_w_bias_sigma() const;
};
} // namespace psim

#endif

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

/** @file psim/fc/flight_computer.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_FC_FLIGHT_COMPUTER_HPP_
#define PSIM_FC_FLIGHT_COMPUTER_HPP_

#include <psim/fc/flight_computer.yml.hpp>

#include <gnc/attitude_controller.hpp>
#include <gnc/attitude_estimator.hpp>
#include <gnc/constants.hpp>

namespace psim {

enum FcState {
  DEPLOYMENT,
  DETUMBLE,
  STANDBY
};

class FlightComputer : public FlightComputerInterface<FlightComputer> {
 private:
  typedef FlightComputerInterface<FlightComputer> Super;

  gnc::AttitudeEstimatorState attitude_estimator_state;
  gnc::AttitudeEstimatorData attitude_estimator_data;
  gnc::AttitudeEstimate attitude_estimate;

  gnc::DetumbleControllerState detumbler_state;
  gnc::DetumbleControllerData detumbler_data;
  gnc::DetumbleActuation detumbler_actuation;

  gnc::PointingControllerState pointer_state;
  gnc::PointingControllerData pointer_data;
  gnc::PointingActuation pointer_actuation;

  Real exit_deployment_on = gnc::constant::nan;

 public:
  using Super::FlightComputerInterface;

  FlightComputer() = delete;
  virtual ~FlightComputer() = default;

  virtual void step() override;

  Real fc_satellite_attitude_P_fro() const;
};
}  // namespace psim

#endif

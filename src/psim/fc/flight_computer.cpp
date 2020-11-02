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

/** @file psim/fc/flight_computer.cpp
 *  @author Kyle Krol
 */

#include <psim/fc/flight_computer.hpp>

#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace psim {

void FlightComputer::step() {
  this->Super::step();

  // Pull in orbit estimates
  auto const &t_s = truth_t_s->get();
  auto const &r_ecef = truth_satellite_orbit_r_ecef->get();

  // Pull in attitude sensor data
  auto const &gyroscope_w = sensors_satellite_gyroscope_w->get();
  auto const &magnetometer_b = sensors_satellite_magnetometer_b->get();
  auto const &sun_sensors_s = sensors_satellite_sun_sensors_s->get();

  // Pull in actuator command state fields
  auto &wheels_t = truth_satellite_wheels_t->get();
  auto &magnetorquers_m = truth_satellite_magnetorquers_m->get();

  // Pull in flight computer outputs
  auto &state = fc_satellite_state.get();
  auto &attitude_q_body_eci = fc_satellite_attitude_q_body_eci.get();
  auto &attitude_w = fc_satellite_attitude_w.get();
  auto &gyroscope_bias = fc_satellite_gyroscope_bias.get();

  // If we currently have a valid attitude estimate
  if (attitude_estimator_state.is_valid) {
    attitude_estimate = gnc::AttitudeEstimate();
    attitude_estimator_data = gnc::AttitudeEstimatorData();

    attitude_estimator_data.t = t_s;
    attitude_estimator_data.r_ecef = r_ecef;
    attitude_estimator_data.b_body = magnetometer_b;
    attitude_estimator_data.s_body = sun_sensors_s;
    attitude_estimator_data.w_body = gyroscope_w;

    gnc::attitude_estimator_update(attitude_estimator_state,
                                   attitude_estimator_data,
                                   attitude_estimate);
  }
  // Otherwise, attempt to initialize the filter
  else {
    gnc::attitude_estimator_reset(attitude_estimator_state,
                                  t_s,
                                  r_ecef,
                                  magnetometer_b,
                                  sun_sensors_s);
  }

  // Output estimates if valid
  if (attitude_estimate.is_valid) {
    attitude_q_body_eci = attitude_estimate.q_body_eci;
    attitude_w = gyroscope_w - attitude_estimate.gyro_bias;
    gyroscope_bias = attitude_estimate.gyro_bias;
  }
  // Otherwise, outputs NaNs
  else {
    attitude_q_body_eci = lin::nans<Vector4>();
    attitude_w = lin::nans<Vector3>();
    gyroscope_bias = lin::nans<Vector3>();
  }

  // Default actuator commands to zero
  wheels_t = lin::zeros<Vector3>();
  magnetorquers_m = lin::zeros<Vector3>();

  // Switch control on the current state
  switch (state) {
    case DEPLOYMENT: {
      // Set exit time if not yet set
      if (!lin::isfinite(exit_deployment_on)) {
        exit_deployment_on = t_s + 30.0 * 60.0;  // 30 minutes later
        break;
      }

      if (t_s >= exit_deployment_on) {
        exit_deployment_on = gnc::constant::nan;
        state = DETUMBLE;
      }
      break;
    }

    case DETUMBLE: {
      // TODO : Implement a stopping condition

      detumbler_actuation = gnc::DetumbleActuation();
      detumbler_data = gnc::DetumbleControllerData();

      detumbler_data.b_body = magnetometer_b;

      gnc::control_detumble(detumbler_state,
                            detumbler_data,
                            detumbler_actuation);
      
      if (lin::all(lin::isfinite(detumbler_actuation.mtr_body_cmd))) {
        magnetorquers_m = detumbler_actuation.mtr_body_cmd;
      }
      
      break;
    }

    case STANDBY: {
      // TODO : Implement this
      break;
    }

    default: {
      throw std::runtime_error("Unexpected flight computer state: " +
                               std::to_string(state));
    }
  };
}

Real FlightComputer::fc_satellite_attitude_P_fro() const {
  return lin::fro(attitude_estimate.P);
}
}  // namespace psim

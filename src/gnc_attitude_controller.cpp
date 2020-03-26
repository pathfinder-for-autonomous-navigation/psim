/** @file gnc_attitude_controller.cpp
 *  @author Kyle Krol */

#include <gnc/attitude_controller.hpp>
#include <gnc/config.hpp>
#include <gnc/constants.hpp>

#include <lin/generators/constants.hpp>

#include <limits>

namespace gnc {

DetumbleControllerState::DetumbleControllerState() {
  this->mtr_body_cmd = lin::nans<decltype(this->mtr_body_cmd)>();
  this->b_body_buffer = CircularBuffer<lin::Vector3f, 10>();
}

DetumbleControllerData::DetumbleControllerData() {
  this->b_body = lin::nans<decltype(this->b_body)>();
}

DetumbleActuation::DetumbleActuation() {
  this->mtr_body_cmd = lin::nans<decltype(this->mtr_body_cmd)>();
}

void control_detumble(DetumbleControllerState &state,
    DetumbleControllerData const &data, DetumbleActuation &actuation) {

  // Default everything to nan
  actuation = DetumbleActuation();

  // We should always get a valid magnetic field reading otherwise the mission
  // is pretty much over
  if (std::isnan(data.b_body(0))) return;

  // Push the new magnetic field into the buffer
  state.b_body_buffer.push(data.b_body);

  // We're ready to calculate a new magnetic torque rod command
  if (state.b_body_buffer.size() == state.b_body_buffer.max_size()) {
    // Look in the middle of the buffer to ignore transients from the previous
    // adjustment to the magnetorquers
    auto size = state.b_body_buffer.size();
    auto db = state.b_body_buffer[size - 1] - state.b_body_buffer[size / 2];
    state.mtr_body_cmd = {
      static_cast<float>(constant::max_mtr_moment) * (db(0) > 0.0f ? -1.0f : 1.0f),
      static_cast<float>(constant::max_mtr_moment) * (db(1) > 0.0f ? -1.0f : 1.0f),
      static_cast<float>(constant::max_mtr_moment) * (db(2) > 0.0f ? -1.0f : 1.0f)
    };
    state.b_body_buffer.clear();
  }

  // Assign the most recently calculated magetorquer command
  actuation.mtr_body_cmd = state.mtr_body_cmd;

}
}  // namespace gnc

//
// src/gnc_attitude_controller.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_attitude_controller.hpp>
#include <gnc_constants.hpp>

#include <limits>

static_assert(std::numeric_limits<double>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");
static_assert(std::numeric_limits<float>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");

namespace gnc {

DetumbleControllerState::DetumbleControllerState() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->b_body = lin::nans<decltype(this->b_body)>();
  this->db_buffer = CircularBuffer<lin::Vector3f, 5>();
}

DetumbleControllerData::DetumbleControllerData() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->b_body = lin::nans<decltype(this->b_body)>();
}

DetumbleActuation::DetumbleActuation() {
  this->mtr_body_cmd = lin::nans<decltype(this->mtr_body_cmd)>();
}

void control_detumble(DetumbleControllerState &state,
    DetumbleControllerData const &data, DetumbleActuation &actuation) {

  // Clear the state if it's outdated (more than ~5 control cycles old)
  if (data.t - state.t > 0.5) state = DetumbleControllerState();

  // Default everything to NaN and ensure a magnetic field vector and time were
  // given
  if (std::isnan(data.b_body(0)) || std::isnan(data.t)) return;

  // If we don't have an old estimate just write the current one into state and
  // wait till the next call to estimate w_body
  if (std::isnan(state.t) || std::isnan(state.b_body(0))) {
    state.t = data.t;
    state.b_body = data.b_body;
  }

  // Calculate db and calculate its moving average
  lin::Vector3f db = (data.b_body - state.b_body) / (data.t - state.t);
  state.db_buffer.push(db);
  db = lin::zeros<float, 3, 1>();
  for (std::size_t i = 0; i < state.db_buffer.size(); i++)
    db = db + state.db_buffer[i];
  db = db / static_cast<float>(state.db_buffer.size());

  // Run the bang-bang controller
  actuation.mtr_body_cmd = {
    constant::max_mtr_moment_f * (db(0) > 0.0f ? -1.0f : 1.0f),
    constant::max_mtr_moment_f * (db(1) > 0.0f ? -1.0f : 1.0f),
    constant::max_mtr_moment_f * (db(2) > 0.0f ? -1.0f : 1.0f)
  };
}
}  // namespace gnc

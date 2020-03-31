/** @file gnc_attitude_controller.cpp
 *  @author Kyle Krol
 *  See MATLAB/adcs/adcs_pointer.c and MATLAB/adcs/adcs_detumbler.m for more
 *  information. */

#include <gnc/attitude_controller.hpp>
#include <gnc/constants.hpp>
#include <gnc/utilities.hpp>

#include <lin/generators/constants.hpp>
#include <lin/references.hpp>

#include <cmath>
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

PointingControllerState::PointingControllerState() { }

PointingControllerData::PointingControllerData()
: primary_desired(lin::nans<decltype(primary_desired)>()),
  primary_current(lin::nans<decltype(primary_current)>()),
  secondary_desired(lin::nans<decltype(secondary_desired)>()),
  secondary_current(lin::nans<decltype(secondary_current)>()),
  w_wheels(lin::nans<decltype(w_wheels)>()),
  w_sat(lin::nans<decltype(w_sat)>()),
  b(lin::nans<decltype(b)>()) { }

PointingActuation::PointingActuation()
: mtr_body_cmd(lin::nans<decltype(mtr_body_cmd)>()),
  rwa_body_cmd(lin::nans<decltype(rwa_body_cmd)>()) { }

static void error_from_quaternion(lin::Vector4f const &q, lin::Vector3f &err) {
  if (q(3) < 0.0f) err = -lin::ref<3, 1>(q, 0, 0);
  else err = lin::ref<3, 1>(q, 0, 0);
}

static void double_objective_error(
    lin::Vector3f const &pri_cur, lin::Vector3f const &pri_des,
    lin::Vector3f const &sec_cur, lin::Vector3f const &sec_des,
    lin::Vector3f &err) {
  lin::Vector4f q;

  // Ensure we are dealing with normal vectors
  GNC_ASSERT_NEAR(1.0f, lin::fro(pri_cur), 1.0e-5f);
  GNC_ASSERT_NEAR(1.0f, lin::fro(pri_des), 1.0e-5f);
  GNC_ASSERT_NEAR(1.0f, lin::fro(sec_cur), 1.0e-5f);
  GNC_ASSERT_NEAR(1.0f, lin::fro(sec_des), 1.0e-5f);

  // Calculate attitude error quaternion using triad and extract error term
  utl::triad(pri_cur, sec_cur, pri_des, sec_des, q);
  error_from_quaternion(q, err);
}

static void single_objective_error(
    lin::Vector3f const &pri_cur, lin::Vector3f const &pri_des,
    lin::Vector3f &err) {
  lin::Vector4f q;

  // Ensure we are dealing with normal vectors
  GNC_ASSERT_NEAR(1.0f, lin::fro(pri_cur), 1.0e-5f);
  GNC_ASSERT_NEAR(1.0f, lin::fro(pri_des), 1.0e-5f);

  // Calculate attitude error quaternion and extract error term
  // Note we have a degree of freedom here that's ignored
  utl::vec_rot_to_quat(pri_des, pri_cur, q);
  error_from_quaternion(q, err);
}

constexpr float sign(float val) {
    return static_cast<float>((0.0f < val) - (val < 0.0f));
}

#ifndef MEX
static
#endif
void mex_control_pointing(PointingControllerState &state,
    PointingControllerData const &data, PointingActuation &actuation, float Kp,
    float Kd, lin::Matrix3x3f const &J) {

  // Default everything to NaN
  actuation = PointingActuation();

  // We need a primary pointing objective and current angular rate at the very
  // least
  if (std::isnan(data.primary_current(0)) ||
      std::isnan(data.primary_desired(0)) || std::isnan(data.w_sat(0)))
    return;

  // No secondary objective is specified
  lin::Vector3f err;
  if (std::isnan(data.secondary_current(0)) ||
      std::isnan(data.secondary_desired(0))) {
    single_objective_error(data.primary_current, data.primary_desired, err);
  }
  // Secondary objective is specified
  else {
    // Check is the primary and secondary objectives are too near
    float thresh = std::cos(10.0f * constant::pi_f / 180.0f);  // within 10 deg.
    if (std::abs(lin::dot(data.secondary_current, data.primary_current)) < thresh ||
        std::abs(lin::dot(data.secondary_desired, data.primary_desired)) < thresh)
      single_objective_error(data.primary_current, data.primary_desired, err);
    else
      double_objective_error(data.primary_current, data.primary_desired,
          data.secondary_current, data.secondary_desired, err);
  }

  // Error term should never be NaN here
  GNC_ASSERT(!std::isnan(err(0)));

  // Determine the wheel actation
  actuation.rwa_body_cmd = -(Kp * err + Kd * data.w_sat);

  // Ensure we have wheel speed data and a magnetic field to perform angular
  // momentum control with
  if (std::isnan(data.w_wheels(0)) || std::isnan(data.b(0))) return;

  // Total angular momentum of the satellite in the body frame
  lin::Vector3f L = J * data.w_sat + constant::J_wheel * data.w_wheels;

  // Total angular momentum should never be NaN here
  GNC_ASSERT(!std::isnan(L(0)));

  // Magnitude of L calls for momentum control
  if (lin::norm(L) > 0.05f * constant::w_wheel_max * constant::J_wheel) {
    L = lin::cross(L, data.b);
    actuation.mtr_body_cmd = constant::max_mtr_moment * lin::Vector3f({
      sign(L(0)), sign(L(1)), sign(L(2)), 
    });
  }
  // No momentum control required at this time
  else
    actuation.mtr_body_cmd = lin::zeros<decltype(actuation.mtr_body_cmd)>();
}

#ifndef MEX
void control_pointing(PointingControllerState &state,
    PointingControllerData const &data, PointingActuation &actuation) {
  mex_control_pointing(state, data, actuation, constant::pointer_Kp,
      constant::pointer_Kd, constant::J_sat);
}
#endif

}  // namespace gnc

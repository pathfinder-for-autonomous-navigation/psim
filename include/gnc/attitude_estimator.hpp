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
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file gnc/attitude_estimator.hpp
 *  @author Kyle Krol
 *
 *  @brief Defines the interface for the attitude estimator.
 *
 *  At a high level, the attitude estimator implemented here is an unscented
 *  Kalman filter estimating both attitude and gyro bias. It's based on "Unscented
 *  Filtering for Spacecraft Attitude Estimation" by John Crassidis and Landis
 *  Markley.
 *
 *  A simple example of how one could use the interface is show here:
 *
 *  ~~~{.cpp}
 *  #include <gnc/attitude_estimator.hpp>
 *  #include <lin/core.hpp>
 *
 *  gnc::AttitudeEstimatorState state; 
 *
 *  void loop(double t, lin::Vector3d const &r_ecef, ..., lin::Vector4f q_eci_body, ...) {
 *    if (state.is_valid) {
 *      gnc::AttitudeEstimatorData data;
 *      data.t = t;
 *      data.r_ecef = r_ecef;
 *      data.b_body = ...
 *      ...
 *
 *      gnc::AttitudeEstimate estimate;
 *      attitude_estimator_update(state, data, estimate);
 *      if (estimate.is_valid) {
 *        // Can perform more checks on estimate.P here too!
 *        q_body_eci = estimate.q_body_eci;
 *        ...
 *      }
 *    }
 *    else {
 *      attitude_estimator_reset(state,...);
 *    }
 *  }
 *  ~~~ */

#ifndef GNC_ATTITUDE_ESTIMATOR_HPP_
#define GNC_ATTITUDE_ESTIMATOR_HPP_

#include <lin/core.hpp>

namespace gnc {
namespace constant {

/** @brief Standard deviation of gyro noise.
 *
 *  This parameter tunes the expected amount of gyro noise within the attitude
 *  estimator. It's defaulted to a value of `1.0e-6f` (units of radians per
 *  second). */
extern float ukf_sigma_v;

/** @brief Standard deviation of gyro bias noise.
 *
 *  This parameter tunes the expected amount of gyro bias noise within the
 *  attitude estimator. It's defaulted to a value of `2.75e-4f` (MKS units). */
extern float ukf_sigma_u;

/** @brief Standard deviation of magnetometer noise.
 *
 *  This parameter tunes the expected amount of magnetometer noise within the
 *  attitude estimator. It's defaulted to a value of `5.0e-7f` (units of Tesla) */
extern float ukf_sigma_b;

/** @brief Standard deviation of sun vector noise (in terms of angle error).
 *
 *  This parameter tunes the expected amount of sun vector noise within the
 *  attitude estimator. It's defaulted to a value of
 *  `2.0f * constant::deg_to_rad_f` (units of radians). */
extern float ukf_sigma_s;

}  // namespace constant

/** @brief Contains the internal state of an attitude estimator.
 *
 *  The internal state of the attitude estimator includes the previous estimates
 *  attitude quaternion, gyro bias estimate, state covariance matrix, timestamp,
 *  and some other variables to act as a buffer for intermediate calculations.
 *
 *  After an `attitude_estimator_*` function call, the `is_valid` member variable
 *  can be read to check if all stored values are finite - i.e. not NaN or
 *  inifinity.
 *
 *  Prior to being used with the `attitude_estimator_update` function, the state
 *  variable must be initialized with a call to `attitude_estimator_reset`. See
 *  more documentation below. */
struct AttitudeEstimatorState {
  /** Variables acting as a calculation buffer
   *  @{ */
  lin::Vectord<6>    x_bar, sigmas[13];
  lin::Vectord<5>    z_bar, measures[13];
  lin::Matrixd<6, 6> P_bar;
  lin::Matrixd<5, 5> P_vv;
  lin::Matrixd<6, 5> P_xy;
  /** @} */
  /** Persistant, state varibles
   *  @{ */
  lin::Vectord<4> q;
  lin::Vectord<6> x;
  lin::Matrixd<6, 6> P;
  double t;
  /** @} */
  /** Default everything to NaN and sets `is_valid` to `false`. */
  AttitudeEstimatorState();
  /** Signals whether the struct specifies a valid filter state. If invalid, all
   *  data members will have been set to NaN. */
  bool is_valid;
};

/** @brief Stores inputs to the `attitude_estimator_update` function.
 *
 *  These inputs include the current time since the PAN epoch (in seconds),
 *  position in ECEF (in meters), magnetic field reading in the body frame (in
 *  Tesla), sun vector reading in the body frame (must be a unit vector), and the
 *  gyroscope angular rate reading in the body frame (units of radians per
 *  second).
 *
 *  In order for a succesfull filter update to be performed, all fields are
 *  required except for the sun vector reading. If no sun vector can be specified,
 *  simply leave it's elements set to NaN. */
struct AttitudeEstimatorData {
  lin::Vector3d r_ecef; //!< Position in ECEF (m).
  lin::Vector3f b_body; //!< Magnetic field reading in the body frame (T).
  lin::Vector3f s_body; //!< Sun vector reading in the body frame (unit vector).
  lin::Vector3f w_body; //!< Angular rate reading in the body frame (rad/s).
  double t;             //!< Time since the PAN epoch (s).
  /** Defaults everything to NaN. */
  AttitudeEstimatorData();
};

/** @brief Stores outputs of the `attitude_estimator_update` function.
 *
 *  These outputs are the current attitude in quaternion representation
 *  (quaternion rotating from ECI to the the body frame), gyro bias estimate in
 *  the body frame (units of radians per second), and the state covariance matrix.
 *  After an `attitude_estimator_*` function call, the `is_valid` member variable
 *  can be read to check if all outputs are finite - i.e. not NaN or infinity.
 *
 *  The gyro bias value can easily be used to determine and "estimated" angular
 *  rate of the spacecraft by simply subtracting the bias vector off of the
 *  measured angular rate.
 *
 *  The state covariance matrix gives the uncertainty in a GRP attitude
 *  representation localized around the current attitude estimate and the gyro
 *  bias. See the filter implementation itself for more information. */
struct AttitudeEstimate {
  lin::Vector4f q_body_eci; //<! Quaternion rotating from ECI to the body frame.
  lin::Vector3f gyro_bias;  //<! Gyro bias (rad/s).
  lin::Matrixf<6, 6> P;     //<! State covariance matrix.
  /** Default everything to NaN and sets `is_valid` to `false`. */
  AttitudeEstimate();
  /** Signals whether the struct specifies a valid attitude estimate. If invalid,
   *  all data members will have been set to NaN. */
  bool is_valid;
};

/** @brief Initializes the attitude filter state.
 *
 *  @param[out] state      Attitude estimator state to be reset/initialized.
 *  @param[in]  t          Time since the PAN epoch (units of seconds).
 *  @param[in]  q_body_eci Quaternion rotating from ECI to the body frame.
 *
 *  Initializes the current time and attitude estimate to the passed arguments.
 *  The gyro bias and covariance is set to default values.
 *
 *  The state will be valid as long as the passed arguments were finite. */
void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector4f const &q_body_eci);

/** @brief Initializes the attitude filter state using the triad method.
 *
 *  @param[out] state  Attitude estimator state to be reset/initialized.
 *  @param[in]  t      Time since the PAN epoch (units of seconds).
 *  @param[in]  r_ecef Position in ECEF (units of meters).
 *  @param[in]  b_body Magnetic field measured in the body frame (units of Tesla).
 *  @param[in]  s_body Sun vector measured in the body frame (unit vector).
 *
 *  Uses the current time, position, b field, and sun vector to calculate an
 *  attitude estimate to initialize the filter with. The other filter state
 *  variables are set to default values.
 *
 *  After calling, it's recommended to check the `state.is_valid` flag to ensure
 *  the estimator state was succesfully reset. The reset call will fail is
 *  parameters given failed the triad algorithm - i.e the sun and magnetic field
 *  vector were nearly parallel.
 *
 *  See the `gnc::utl::triad` documentation for more information. */
void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector3d const &r_ecef, lin::Vector3f const &b_body,
    lin::Vector3f const &s_body);

/** @brief Updates the attitude estimate given a set of sensor readings.
 *
 *  @param[inout] state    Previous filter state.
 *  @param[in]    data     Sensor readings.
 *  @param[out]   estimate Updated attitude estimate.
 * 
 *  Calculates an updated attitude estimate using one of two unscented Kalman
 *  filter implementations. The first runs if all required elements of the `data`
 *  struct as well an a sun vector readings are specified; it's the superior
 *  filter. The latter runs when no sun vector is specified. The estimator state
 *  and estimate will default to invalid if some expected input isn't provided.
 *
 *  After calling, it's recommended to check the `state.is_valid` or
 *  `estiamte.is_valid` field to ensure the update step completed succesfully.
 *  Further checks on the updated covariance matrix are also suggested to check
 *  for runaway uncertainties.
 *
 *  An "invalid" input will result in invalid outputs. */
void attitude_estimator_update(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate);

}  // namespace gnc

#endif

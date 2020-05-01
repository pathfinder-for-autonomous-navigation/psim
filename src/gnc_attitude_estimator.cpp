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

/** @file gnc_attitude_estimator.cpp
 *  @author Kyle Krol */

#include <gnc/attitude_estimator.hpp>
#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/factorizations/chol.hpp>
#include <lin/factorizations/qr.hpp>
#include <lin/generators/constants.hpp>
#include <lin/generators/identity.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>
#include <lin/references.hpp>
#include <lin/substitutions/backward_substitution.hpp>

namespace gnc {
namespace constant {

GNC_TRACKED_CONSTANT(float, ukf_sigma_v, 1.0e-6f);
GNC_TRACKED_CONSTANT(float, ukf_sigma_u, 2.75e-4f);
GNC_TRACKED_CONSTANT(float, ukf_sigma_b, 5.0e-7f);
GNC_TRACKED_CONSTANT(float, ukf_sigma_s, 2.0f * constant::deg_to_rad_f);

}  // namespace constant

/** Specifies the floating point type used for most internal UKF calculations. */
typedef double ukf_float;

/** Useful type definitions for the filter implementation.
 *  @{ */
typedef lin::Vector<ukf_float, 2> UkfVector2;
typedef lin::Vector<ukf_float, 3> UkfVector3;
typedef lin::Vector<ukf_float, 4> UkfVector4;
typedef lin::Vector<ukf_float, 5> UkfVector5;
typedef lin::Vector<ukf_float, 6> UkfVector6;
typedef lin::Matrix<ukf_float, 3, 3> UkfMatrix3x3;
typedef lin::Matrix<ukf_float, 4, 4> UkfMatrix4x4;
typedef lin::Matrix<ukf_float, 5, 5> UkfMatrix5x5;
typedef lin::Matrix<ukf_float, 5, 6> UkfMatrix5x6;
typedef lin::Matrix<ukf_float, 6, 5> UkfMatrix6x5;
typedef lin::Matrix<ukf_float, 6, 6> UkfMatrix6x6;
/** @} */

/** @fn ukf_propegate
 *  Propegates forward the attitude of a rotating rigid body.
 *
 *  @param[in]  dt    Timestep (seconds).
 *  @param[in]  w     Angular rate (radians per second).
 *  @param[in]  q_old Initial attitude.
 *  @param[out] q_new Final attitude.
 *
 *  This function assuming the body is rotating at a constant angular rate and
 *  therefore may only be accurate for small timesteps. */
static void ukf_propegate(ukf_float dt, UkfVector3 const &w,
    UkfVector4 const &q_old, UkfVector4 &q_new) {
  GNC_ASSERT_NORMALIZED(q_old);

  // Calculate transition matrix
  UkfMatrix4x4 O;
  {
    ukf_float norm_w = lin::norm(w);
    UkfVector3 psi = lin::sin(0.5f * dt * norm_w) * w / norm_w;
    UkfMatrix3x3 psi_x = {
      ukf_float(0.0),         -psi(2),          psi(1),
              psi(2),  ukf_float(0.0),         -psi(0),
             -psi(1),          psi(0),  ukf_float(0.0)
    };
    O = lin::cos(0.5f * norm_w * dt) * lin::identity<ukf_float, 4>();
    lin::ref<3, 3>(O, 0, 0) = lin::ref<3, 3>(O, 0, 0) - psi_x;
    lin::ref<3, 1>(O, 0, 3) = psi;
    lin::ref<1, 3>(O, 3, 0) = -lin::transpose(psi);
  }

  // Propegate our quaternion forward and normalize to be safe
  q_new = O * q_old;
  q_new = q_new / lin::norm(q_new);
}

/** @fn ukf
 *  Performs a single attitude estimator update step.
 *
 *  @param[inout] state             Attitude estimator state.
 *  @param[in]    data              Input sensor measurements.
 *  @param[in]    ukf_kalman_update Kalman gain update step function.
 * 
 *  This functions takes `state.q`, `state.x`, `state.t`, and `data.*` (with the
 *  potential exception of `data.s_body`) as inputs. It's assumed that the values
 *  in both structs are finite.
 * 
 *  This function will determine sigma points, propegate sigma points, simulate
 *  expected measurements, and populate the `x_bar`, `z_bar`, `P_bar`, `P_vv`,
 *  and `P_xy` fields of the estimator state.
 * 
 *  The `ukf_kalman_update` function will then be called and is responsible for
 *  calculating the Kalman gain and updating the estimator state vector
 *  (`state.x`) and covariance (`state.P`). This was done so the magnetometer only
 *  and magnetomter plus sun vector filters could share a large amount of code.
 * 
 *  Once the function returns, `ukf` will update `state.t`, zero the first three
 *  components of `state.x` (zeroth sigma point has "no" attitude error), and
 *  update `state.q`. */
static void ukf(AttitudeEstimatorState &state, AttitudeEstimatorData const &data,
    void (*ukf_kalman_update)(AttitudeEstimatorState &state, AttitudeEstimatorData const &data)) {
  /** Tuning parameter for the shape of the sigma point distribution. */
  constexpr static ukf_float lambda = 1.0;
  /** GRP conversion parameters. @{ */
  constexpr static ukf_float a = 1.0;
  constexpr static ukf_float f = 2.0 * (a + 1.0);
  /** @} 
   *  Length of a state vector. */
  constexpr static ukf_float N = 6.0;

  // Timestep
  ukf_float dt;
  {
    dt = data.t - state.t;
    /** Smallest possible timestep we'll allow this filter to take. */
    GNC_TRACKED_CONSTANT(constexpr static ukf_float, dt_thresh, 1.0e-3);
    /* This check is to ensure we don't initialize and call update on the filter
     * in the same timestep. */
    if (dt < dt_thresh) return;
  }

  // Process noise covariance
  UkfMatrix6x6 Q = lin::zeros<UkfMatrix6x6>();
  {
    /** Tuning factor scaling the process noise covariance matrix. */
    GNC_TRACKED_CONSTANT(constexpr static ukf_float, Q_factor, 1.0e3);

    ukf_float factor = Q_factor * dt / 2.0f;
    ukf_float var_u = constant::ukf_sigma_u * constant::ukf_sigma_u;
    ukf_float var_v = constant::ukf_sigma_v * constant::ukf_sigma_v;
    Q(0, 0) = factor * (var_v - var_u * dt * dt / 6.0f);
    Q(1, 1) = Q(0, 0);
    Q(2, 2) = Q(0, 0);
    Q(3, 3) = factor * var_u;
    Q(4, 4) = Q(3, 3);
    Q(5, 5) = Q(3, 3);
  }

  // Generate sigma points
  {
    UkfMatrix6x6 L = state.P + Q;
    lin::chol(L);

    state.sigmas[0] = state.x;
    L = lin::sqrt(N + lambda) * L;
    for (lin::size_t i = 0; i < L.cols(); i++) state.sigmas[i + 1] = state.x + lin::ref_col(L, i);
    for (lin::size_t i = 0; i < L.cols(); i++) state.sigmas[i + L.cols() + 1] = state.x - lin::ref_col(L, i);
  }

  // Propegate the center sigma points attitude
  UkfVector4 q_new;
  {
    UkfVector3 w = data.w_body - lin::ref<3, 1>(state.x, 3, 0);
    ukf_propegate(dt, w, state.q, q_new);
  }

  // Propegate sigma points forward and calculate expected measurements
  {
    // Generate expected sun and magnetic field vectors
    UkfVector3 s_exp, b_exp;
    {
      env::sun_vector(data.t, s_exp); // s_exp in ECI

      UkfVector4 q_eci_ecef;
      env::earth_attitude(data.t, q_eci_ecef); // q_eci_ecef = q_ecef_eci
      utl::quat_conj(q_eci_ecef);              // q_eci_ecef = q_eci_ecef

      env::magnetic_field(data.t, data.r_ecef, b_exp); // b_exp in ECEF
      utl::rotate_frame(q_eci_ecef, b_exp);            // b_exp in ECI
    }

    // Calculate the expected measurements for the zeroth sigma point
    {
      UkfVector3 s, b;
      utl::rotate_frame(q_new, s_exp, s); // s in the body frame
      utl::rotate_frame(q_new, b_exp, b); // b in the body frame

      state.measures[0] = {
        lin::atan(s(1) / s(0)),
        lin::acos(s(2)),
        b(0),
        b(1),
        b(2)
      };
    }

    // Conjugate of the q_new
    UkfVector4 conj_q_new;
    utl::quat_conj(q_new, conj_q_new);

    // Propegate and generate expected measurements for the other sigma points
    for (lin::size_t i = 1; i < 13; i++) {
      // Calculate this sigma's propegated attitude
      UkfVector4 _q_new;
      {
        UkfVector4 q, _q_old;
        utl::grp_to_quat(lin::ref<3, 1>(state.sigmas[i], 0, 0).eval(), a, f, q);
        utl::quat_cross_mult(q, UkfVector4(state.q), _q_old);

        UkfVector3 w = data.w_body - lin::ref<3, 1>(state.sigmas[i], 3, 0);
        ukf_propegate(dt, w, _q_old, _q_new);
      }

      // Determine expected measurements
      {
        UkfVector3 s, b;
        utl::rotate_frame(_q_new, s_exp, s); // s in the body frame
        utl::rotate_frame(_q_new, b_exp, b); // b_exp in the body frame

        state.measures[i] = {
          lin::atan(s(1) / s(0)),
          lin::acos(s(2)),
          b(0),
          b(1),
          b(2)
        };
      }

      // Determine the propegated sigmas
      {
        lin::Vector4d q;
        utl::quat_cross_mult(_q_new, conj_q_new, q); // q = "residual" propegated rotation

        lin::Vector3d p;
        utl::quat_to_qrp(q, a, f, p);
        lin::ref<3, 1>(state.sigmas[i], 0, 0) = p;
      }
    }
  }

  // Calculate mean expected state, mean expected measurements, and associated
  // covariances
  {
    UkfVector6   &x_bar = state.x_bar;
    UkfVector5   &z_bar = state.z_bar;
    UkfMatrix6x6 &P_bar = state.P_bar;
    UkfMatrix5x5 &P_vv  = state.P_vv;
    UkfMatrix6x5 &P_xy  = state.P_xy;

    // Calculate sigma point and covariance weights
    ukf_float weight_c = lambda / (N + lambda);
    ukf_float weight_o = 1.0f / (2.0f * (N + lambda));

    // Calculate x_bar and z_bar
    x_bar = weight_c * state.sigmas[0];
    z_bar = weight_c * state.measures[0];
    for (lin::size_t i = 1; i < 13; i++) {
      x_bar = x_bar + weight_o * state.sigmas[i];
      z_bar = z_bar + weight_o * state.measures[i];
    }

    // Plus the associated covariances
    UkfVector6 dx1 = state.sigmas[0] - x_bar;
    UkfVector5 dz1 = state.measures[0] - z_bar;
    P_bar = P_bar + weight_c * dx1 * lin::transpose(dx1);
    P_vv = P_vv + weight_c * dz1 * lin::transpose(dz1);
    P_xy = P_xy + weight_c * dx1 * lin::transpose(dz1);
    for (lin::size_t i = 1; i < 13; i++) {
      dx1 = state.sigmas[i] - x_bar;
      dz1 = state.measures[i] - z_bar;
      P_bar = P_bar + weight_o * (dx1 * lin::transpose(dx1));
      P_vv = P_vv + weight_o * (dz1 * lin::transpose(dz1));
      P_xy = P_xy + weight_o * (dx1 * lin::transpose(dz1));
    }

    // Sensor noise covariance
    UkfMatrix5x5 R = lin::zeros<UkfMatrix5x5>();
    {
      R(0, 0) = constant::ukf_sigma_s * constant::ukf_sigma_s;
      R(1, 1) = R(0, 0);
      R(2, 2) = constant::ukf_sigma_b * constant::ukf_sigma_b;
      R(3, 3) = R(2, 2);
      R(4, 4) = R(2, 2);
    }

    P_bar = P_bar + Q; // Add process noise to predicted covariance
    P_vv = P_vv + R;   // Add sensor noise to innovation covariance
  }

  // Kalman gain step
  ukf_kalman_update(state, data);

  // Process x (which is now x_new) and P (which is now P_new)
  {
    // Update time
    state.t = data.t;

    // Perturb q_new according to the new state
    lin::Vector4d q;
    utl::grp_to_quat(lin::ref<3, 1>(state.x, 0, 0).eval(), a, f, q);
    utl::quat_cross_mult(q, q_new, state.q);

    // Reset the attitude portion of the state to zeros
    lin::ref<3, 1>(state.x, 0, 0) = lin::zeros<UkfVector3>();
  }
}

/** @fn ukf_m
 *  Update attitude estimator state given a magnetometer reading.
 * 
 *  @param[inout] state Attitude filter state.
 *  @param[in]    data  Input sensor data. */
static void ukf_m(AttitudeEstimatorState &state, AttitudeEstimatorData const &data) {
  // TODO : Implement this
  state = AttitudeEstimatorState();
}

/** @fn ukf_ms
 *  Update attitude estimator state given magnetometer and sun vector readings.
 * 
 *  @param[inout] state Attitude filter state.
 *  @param[in]    data  Input sensor data. */
static void ukf_ms(AttitudeEstimatorState &state, AttitudeEstimatorData const &data) {
  ukf(state, data, [](AttitudeEstimatorState &state, AttitudeEstimatorData const &data) -> void {
    // Calculate Kalman gain
    UkfMatrix6x5 K;
    {
      UkfMatrix5x5 Q, R;
      lin::qr(state.P_vv, Q, R);
      lin::backward_sub(R, Q, lin::transpose(Q).eval()); // Q = inv(P_yy)    
      K = state.P_xy * Q;
    }

    // Calculate this steps measurement
    UkfVector5 z_new {
      lin::atan(data.s_body(1) / data.s_body(0)),
      lin::acos(data.s_body(2)),
      data.b_body(0),
      data.b_body(1),
      data.b_body(2)
    };

    // Update the state vector and covariance
    state.x = state.x_bar + K * (z_new - state.z_bar).eval();
    state.P = state.P_bar - K * (state.P_vv * lin::transpose(K)).eval();
  });
}

AttitudeEstimatorState::AttitudeEstimatorState()
: q(lin::nans<UkfVector4>()),
  x(lin::nans<UkfVector6>()),
  P(lin::nans<UkfMatrix6x6>()),
  t(constant::nan),
  is_valid(false) { }

AttitudeEstimatorData::AttitudeEstimatorData()
: r_ecef(lin::nans<lin::Vector3d>()),
  b_body(lin::nans<lin::Vector3f>()),
  s_body(lin::nans<lin::Vector3f>()),
  w_body(lin::nans<lin::Vector3f>()),
  t(constant::nan) { }

AttitudeEstimate::AttitudeEstimate()
: q_body_eci(lin::nans<lin::Vector4f>()),
  gyro_bias(lin::nans<lin::Vector3f>()),
  P(lin::nans<lin::Matrixf<6, 6>>()),
  is_valid(false) { }

void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector4f const &q_body_eci) {
  GNC_ASSERT_NORMALIZED(q_body_eci);

  /* Default, initial attitude covariance (units of radiancs squared). */
  GNC_TRACKED_CONSTANT(constexpr static float, var_q, 0.0305);
  /* Default, initial gyro bias covariance (units of radians per second all
   * squared). */
  GNC_TRACKED_CONSTANT(constexpr static float, var_g, 0.0049);
  /* Default, initial state. */
  GNC_TRACKED_CONSTANT(constexpr static UkfVector6, init_state, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Set the proper fields in state
  state.t = t;
  state.q = q_body_eci;
  state.x = init_state;
  state.P = lin::zeros<decltype(state.P)>();
  state.P(0, 0) = var_q;
  state.P(1, 1) = state.P(0, 0);
  state.P(2, 2) = state.P(0, 0);
  state.P(3, 3) = var_g;
  state.P(4, 4) = state.P(3, 3);
  state.P(5, 5) = state.P(4, 4);
  state.is_valid = true;

  // If invalid, set everything back to NaNs
  if (!lin::isfinite(state.t) ||
      !lin::all(lin::isfinite(state.q)) ||
      !lin::all(lin::isfinite(state.x)) ||
      !lin::all(lin::isfinite(state.P)))
    state = AttitudeEstimatorState();
}

void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector3d const &r_ecef, lin::Vector3f const &b_body,
    lin::Vector3f const &s_body) {
  GNC_ASSERT_NORMALIZED(s_body);

  // TODO : Implement this
  // https://github.com/pathfinder-for-autonomous-navigation/psim/issues/187
  lin::Vector4f q_body_eci = lin::nans<float, 4, 1>();
  attitude_estimator_reset(state, t, q_body_eci);
}

void attitude_estimator_update(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate) {
  // Ensure we have a valid state and all the required inputs
  if (!state.is_valid ||
      !lin::all(lin::isfinite(data.r_ecef)) ||
      !lin::all(lin::isfinite(data.b_body)) ||
      !lin::all(lin::isfinite(data.w_body)) ||
      !lin::isfinite(data.t)) {
    // Set the state and estimate to be invalid
    state = AttitudeEstimatorState();
    estimate = AttitudeEstimate();
    return;
  }

  GNC_ASSERT(lin::all(lin::isfinite(state.q)));
  GNC_ASSERT(lin::all(lin::isfinite(state.x)));
  GNC_ASSERT(lin::all(lin::isfinite(state.P)));
  GNC_ASSERT_NORMALIZED(state.q);

  // Run the magnetomter and sun vector implementation
  if (lin::all(lin::isfinite(data.s_body))) {
    GNC_ASSERT_NORMALIZED(data.s_body);
    ukf_ms(state, data);
  }
  // Run the magnetometer only implementation
  else {
    ukf_m(state, data);
  }

  // Update resulted in an invalid state
  if (!lin::all(lin::isfinite(state.q)) ||
      !lin::all(lin::isfinite(state.x)) ||
      !lin::all(lin::isfinite(state.P)) ||
      !lin::isfinite(state.t)) {
    state = AttitudeEstimatorState();
    estimate = AttitudeEstimate();
  }
  // Update gave a valid output
  else {
    estimate.q_body_eci = state.q;
    estimate.gyro_bias = lin::ref<3, 1>(state.x, 3, 0);
    estimate.P = state.P;
    estimate.is_valid = true;
  }
}
}  // namespace gnc

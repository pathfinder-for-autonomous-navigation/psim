/** @file gnc_attitude_estimator.cpp
 *  @author Kyle Krol */

#include <gnc/attitude_estimator.hpp>
#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <lin/generators/constants.hpp>

#include <cmath>
#include <limits>

namespace gnc {

AttitudeEstimatorState::AttitudeEstimatorState() { }

AttitudeEstimatorData::AttitudeEstimatorData()
: t(std::numeric_limits<double>::quiet_NaN()),
  r_ecef(lin::nans<decltype(r_ecef)>()),
  b_body(lin::nans<decltype(b_body)>()),
  s_body(lin::nans<decltype(s_body)>()),
  w_body(lin::nans<decltype(w_body)>()) { }

AttitudeEstimate::AttitudeEstimate()
: q_body_eci(lin::nans<decltype(q_body_eci)>()),
  w_body(lin::nans<decltype(w_body)>()) { }

/** @fn estimate_q_body_eci */
static int estimate_q_body_eci(double t, lin::Vector3d const &r_ecef,
    lin::Vector3f const &s_body, lin::Vector3f const &b_body,
    lin::Vector4f &q_body_eci) {
  lin::Vector3f s_eci, b_eci;

  // Determine the expected magnetic field
  env::magnetic_field(t, lin::Vector3f(r_ecef), b_eci);

  // Ensure the measured and expected magnetic field are large enough
  float thresh = constant::b_noise_floor * constant::b_noise_floor;
  if ((lin::fro(b_eci) < thresh) || (lin::fro(b_body) < thresh)) {
    return;
  }

  // Determine the expected sun vector
  env::sun_vector(t, s_eci);

  // Ensure the sun vectors are unit vectors
  // GNC_ASSERT_NEAR(1.0f, lin::fro(s_body), 1.0e-5f); // TODO : waiting on rebase
  // GNC_ASSERT_NEAR(1.0f, lin::fro(s_eci), 1.0e-5f);

  // Perform triad
  // Remember the value of q_body_eci won't be changed if triad fails
  utl::triad(s_eci, (b_eci / lin::norm(b_eci)).eval(), s_body,
      (b_body / lin::norm(b_body)).eval(), q_body_eci);
}

void estimate_attitude(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate) {

  // Default everything to NaN
  estimate = AttitudeEstimate();

  // Ensure the magnetic field vector, sun vector, ECEF position, and time were
  // given
  if (std::isnan(data.b_body(0)) || std::isnan(data.s_body(0)) ||
      std::isnan(data.r_ecef(0)) || std::isnan(data.t))
    return;

  // Perform triad and pass the angular rate value through
  estimate_q_body_eci(data.t, data.r_ecef, data.s_body, data.b_body, estimate.q_body_eci);
  estimate.w_body = data.w_body;

}
}  // namespace gnc

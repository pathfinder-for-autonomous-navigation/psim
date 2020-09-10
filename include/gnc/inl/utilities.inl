/** @file gnc/inl/utilities.inl
 *  @author Kyle Krol */

#include "../utilities.hpp"

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace gnc {
namespace utl {

template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> const &q, lin::Vector<T, 4> &res) {
  res = { -q(0), -q(1), -q(2), q(3) };
}

template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> &q) {
  quat_conj(q, q);
}

template <typename T>
constexpr void quat_cross_mult(lin::Vector<T, 4> const &q1, lin::Vector<T, 4> const &q2,
    lin::Vector<T, 4> &res) {
  auto const qv1 = lin::ref<3, 1>(q1, 0, 0);
  auto const qv2 = lin::ref<3, 1>(q2, 0, 0);
  lin::ref<3, 1>(res, 0, 0) = lin::cross(qv2, qv1) + q1(3) * qv2 + q2(3) * qv1;
  res(3) = q1(3) * q2(3) - lin::dot(qv1, qv2);
}

template <typename T>
constexpr void quat_to_qrp(lin::Vector<T, 4> const &q, T a, T f, lin::Vector<T, 3> &p) {
  p = lin::ref<3, 1>(q, 0, 0) * (f / (a + q(3)));
}

template <typename T>
constexpr void grp_to_quat(lin::Vector<T, 3> const &p, T a, T f, lin::Vector<T, 4> &q) {
  T fro_p = lin::fro(p);
  T sqr_f = f * f;
  q(3) = (f * std::sqrt(sqr_f + (1 - a * a) * fro_p) - a * fro_p) / (sqr_f + fro_p);
  lin::ref<3, 1>(q, 0, 0) = p * ((a + q(3)) / f);
}

template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q, lin::Vector<T, 3> const &v,
    lin::Vector<T, 3> &res) {
  auto const qv = lin::ref<3, 1>(q, 0, 0);
  res = v + lin::cross(
      (static_cast<T>(2.0) * qv).eval(),
      (lin::cross(qv, v) - q(3) * v).eval()
    );
}

template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q, lin::Vector<T, 3> &v) {
  rotate_frame(q, v, v);
}

template <typename T>
constexpr void dcm(lin::Matrix<T, 3, 3> &DCM, lin::Vector<T, 3> const &x,
    lin::Vector<T, 3> const &y) {
  // Check if x and y are parallel or antiparallel
  if (std::abs(lin::dot(x / lin::norm(x), y / lin::norm(y))) > T(0.999)) {
    DCM = lin::nans<lin::Matrix<T, 3, 3>>();
    return;
  }

  // Generate references to our unit vector destinations
  auto x_hat = lin::ref_row(DCM, 0);
  auto y_hat = lin::ref_row(DCM, 1);
  auto z_hat = lin::ref_row(DCM, 2);
  // Set x_hat in place
  x_hat = lin::transpose(x);
  x_hat = x_hat / lin::norm(x_hat);
  // Set y_hat in place
  y_hat = lin::transpose(y);
  y_hat = y_hat - x_hat * lin::dot(x_hat, y_hat);
  y_hat = y_hat / lin::norm(y_hat);
  // Set z_hat in place
  z_hat = lin::cross(x_hat, y_hat);

  // Assert that DCM is a direction cosine matrix
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 0));
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 1));
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 2));
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 0), lin::ref_col(DCM, 1)), 1.0e-3f);
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 0), lin::ref_col(DCM, 2)), 1.0e-3f);
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 1), lin::ref_col(DCM, 2)), 1.0e-3f);
}

// TODO : Potentially add diagonal references to lin
template <typename T>
constexpr void dcm_to_quat(lin::Matrix<T, 3, 3> const &DCM,
    lin::Vector<T, 4> &q) {
  // Assert that DCM is a direction cosine matrix
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 0));
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 1));
  GNC_ASSERT_NORMALIZED(lin::ref_col(DCM, 2));
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 0), lin::ref_col(DCM, 1)), 1.0e-3f);
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 0), lin::ref_col(DCM, 2)), 1.0e-3f);
  GNC_ASSERT_NEAR(0.0f, lin::dot(lin::ref_col(DCM, 1), lin::ref_col(DCM, 2)), 1.0e-3f);

  // Find the maximum among the diagonal elements and trace
  T max = DCM(0, 0);
  T trace = DCM(0, 0);
  lin::size_t idx = 0;
  for (lin::size_t i = 1; i < 3; i++) {
    trace += DCM(i, i);
    if (DCM(i, i) > max) {
      max = DCM(i, i);
      idx = i;
    }
  }
  if (trace > max) {
    max = trace;
    idx = 3;
  }

  // Calculate the unnormalized quaternion
  switch (idx) {
    // Max was M(0, 0)
    case 0:
      q = {
        static_cast<T>(2.0) * max - trace + static_cast<T>(1.0),
        DCM(0, 1) + DCM(1, 0),
        DCM(0, 2) + DCM(2, 0),
        DCM(1, 2) - DCM(2, 1)
      };
      break;
    // Max was M(1, 1)
    case 1:
      q = {
        DCM(1, 0) + DCM(0, 1),
        static_cast<T>(2.0) * max - trace + static_cast<T>(1.0),
        DCM(1, 2) + DCM(2, 1),
        DCM(2, 0) - DCM(0, 2)
      };
      break;
    // Max was M(2, 2)
    case 2:
      q = {
        DCM(2, 0) + DCM(0, 2),
        DCM(2, 1) + DCM(1, 2),
        static_cast<T>(2.0) * max - trace + static_cast<T>(1.0),
        DCM(0, 1) - DCM(1, 0)
      };
      break;
    // Max was trace(M)
    default:
      q = {
        DCM(1, 2) - DCM(2, 1),
        DCM(2, 0) - DCM(0, 2),
        DCM(0, 1) - DCM(1, 0),
        trace + static_cast<T>(1.0)
      };
  }

  // Normalize the quaternion
  q = q / lin::norm(q);
}

template <typename T>
constexpr void triad(lin::Vector<T, 3> const &R1, lin::Vector<T, 3> const &R2,
    lin::Vector<T, 3> const &r1, lin::Vector<T, 3> const &r2,
    lin::Vector<T, 4> &q) {
  // Ensure we are working with normal vectors
  GNC_ASSERT_NORMALIZED(R1);
  GNC_ASSERT_NORMALIZED(R2);
  GNC_ASSERT_NORMALIZED(r1);
  GNC_ASSERT_NORMALIZED(r2);

  // Default the returned value to NaN
  q = lin::nans<lin::Vector<T, 4>>();

  // Ensure that sun and magnetic field vectors aren't parallel or anitparallel
  T thresh = std::cos(1.0f * constant::deg_to_rad);
  if ((lin::abs(lin::dot(R1, R2)) > thresh) || (lin::abs(lin::dot(r1, r2)) > thresh))
    return;

  // Calculate right handed bases for the known frame
  lin::Vector<T, 3> const &n1 = R1;
  lin::Vector<T, 3> const  n2 = lin::cross(n1, R2) / lin::norm(lin::cross(n1, R2));
  lin::Vector<T, 3> const  n3 = lin::cross(n1, n2);

  // Calculate right handed bases for measurements' frame
  lin::Vector<T, 3> const &b1 = r1;
  lin::Vector<T, 3> const  b2 = lin::cross(b1, r2) / lin::norm(lin::cross(b1, r2));
  lin::Vector<T, 3> const  b3 = lin::cross(b1, b2);

  // Calculate the DCM between the two frames and extract the quaternion
  lin::Matrix<T, 3, 3> const Q =
      b1 * lin::transpose(n1) + b2 * lin::transpose(n2) + b3 * lin::transpose(n3);
  dcm_to_quat(Q, q);
}

// TODO : More strict tolerancing to determine 'antiparallel'
template <typename T>
constexpr void vec_rot_to_quat(lin::Vector<T, 3> const &u,
    lin::Vector<T, 3> const &v, lin::Vector<T, 4> &q) {
  // Ensure we are working with normal vectors
  GNC_ASSERT_NORMALIZED(u);
  GNC_ASSERT_NORMALIZED(v);

  // Handle u and v being near anitparallel
  T x = lin::dot(u, v);
  if (std::abs(x + static_cast<T>(1)) < static_cast<T>(0.0001)) {
    q = { static_cast<T>(0), static_cast<T>(0), static_cast<T>(1), static_cast<T>(0) };
  }
  // Handle the general case
  else {
    q(3) = sqrt((static_cast<T>(1) + x) / static_cast<T>(2));
    lin::ref<3, 1>(q, 0, 0) = lin::cross(u, v) / (static_cast<T>(2) * q(3));
  }
}
}  // namespace utl
}  // namespace gnc

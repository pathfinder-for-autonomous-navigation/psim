/** @file gnc_ode.cpp
 *  @author Kyle Krol */

#include <gnc/config.hpp>
#include <gnc/ode.hpp>

#include <algorithm>
#include <cmath>

namespace gnc {

// Reference:
//  https://en.wikipedia.org/wiki/Bogacki–Shampine_method
template <typename T>
int ode23(T ti, T tf, T const *yi, T *yf, unsigned int ne, T *bf, T h_min,
    T rel_tol, T abs_tol, unsigned int max_iter, void (*const f)(T, T const *, T *)) {
  // Table of values
  constexpr static T a21 = 1.0L / 2.0L, a32 = 3.0L / 4.0L, a41 = 2.0L / 9.0L,
      a42 = 1.0L / 3.0L, a43 = 4.0L / 9.0L;
  constexpr static T bs1 = 7.0L / 24.0L, bs2 = 1.0L / 4.0L, bs3 = 1.0L / 3.0L, bs4 = 1.0L / 8.0L;
  constexpr static T c2 = 1.0L / 2.0L, c3 = 3.0L / 4.0L;

 // Setup scratch buffers
  T *const k1 = bf;
  T *const k2 = k1 + ne;
  T *const k3 = k2 + ne;
  T *const k4 = k3 + ne;
  T *const ks = k4 + ne;
  T *const kz = ks + ne;

  // Initialize local variables
  int err = ODE_ERR_OK;
  unsigned int iter = 0;
  T h_next = (tf - ti) / static_cast<T>(10);
  T t = ti;
  T delta_max;
  T h;

  // Copy yi into yf and ensure we still have some time to integrate over
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i];
  if (ti >= tf) return err | ODE_ERR_BAD_INTERVAL;

  // Calculate the initial k1
  f(t, yf, k1);

  for (;;) {

    // Check iteration bound
    if (iter++ > max_iter) return err | ODE_ERR_MAX_ITER;

    for (;;) {  // Start error checking loop
    
      // Setup
      h = h_next;

      // Step forward
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * a21 * k1[i];
      f(t + c2 * h, ks, k2);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * a32 * k2[i];
      f(t + c3 * h, ks, k3);
      for (unsigned int i = 0; i < ne; i++)  // Third order solution
        ks[i] = yf[i] + h * (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]);
      f(t + h, ks, k4);
      for (unsigned int i = 0; i < ne; i++)  // Second order solution
        kz[i] = yf[i] + h * (bs1 * k1[i] + bs2 * k2[i] + bs3 * k3[i] + bs4 * k4[i]);

      // Calculate largest error
      delta_max = static_cast<T>(0);
      for (unsigned int i = 0; i < ne; i++) {
        T delta = std::abs(ks[i] - kz[i]) /
            std::max(abs_tol, rel_tol * std::max(std::abs(ks[i]), std::abs(kz[i])));
        if (delta > delta_max) delta_max = delta;
      }

      // Helpfull constants
      constexpr static T one = static_cast<T>(1), three = static_cast<T>(3),
          one_third = one / three;
      
      // Extremely small error (prevent divide by zero and bump step size)
      if (delta_max < static_cast<T>(1e-20)) {
        h_next = h * three;
      }
      // Small error (bump step size a little)
      else if (delta_max < one_third) {
        h_next = h * std::pow(one / delta_max, one_third);
      }
      // Large error (shrink step size, ensure it large enough, and continue)
      else if (delta_max > one) {
        h_next = h * one_third;
        if (h_next < h_min) {
          h_next = h_min;
          err |= ODE_ERR_MIN_STEP;
        }
        continue;
      }
      // Step size is working well (don't calculate the next k1 vector)
      else {
        h_next = h;
      }

      // Exit loop to update yf and t
      break;

    }  // End errror checking loop

    // Step forward time, see if we're done, and prevent to large of a step
    t += h;
    for (unsigned int i = 0; i < ne; i++) yf[i] = ks[i];
    for (unsigned int i = 0; i < ne; i++) k1[i] = k4[i];
    if (t >= tf) return err;
    if (t + h_next >= tf) h_next = tf - t;
    
  }
}

GNC_ODEXX_TEMPLATE(ode23, float);
GNC_ODEXX_TEMPLATE(ode23, double);

// Reference:
//  https://en.wikipedia.org/wiki/Dormand–Prince_method
//  Earl Kirkland example code and notes from AEP 4380
//  Numerical Recipes 3rd Edition: The Art of Scientific Computing
template <typename T>
int ode45(T ti, T tf, T const *yi, T *yf, unsigned int ne, T *bf, T h_min,
    T rel_tol, T abs_tol, unsigned int max_iter, void (*const f)(T, T const *, T *)) {
  // Table of values
  constexpr static T a21 = 1.0 / 5.0,
      a31 = 3.0 / 40.0, a32 = 9.0 / 40.0,
      a41 = 44.0 / 45.0, a42 = -56.0 / 15.0, a43 = 32.0 / 9.0,
      a51 = 19372.0 / 6561.0, a52 = -25360.0 / 2187.0,
        a53 = 64448.0 / 6561.0, a54 = -212.0 / 729.0,
      a61 = 9017.0 / 3168.0, a62 = -355.0 / 33.0, a63 = 46732.0 / 5247.0,
        a64 = 49.0 / 176.0, a65 = -5103.0 / 18656.0,
      a71 = 35.0 / 384.0, a73 = 500.0 / 1113.0, a74 = 125.0 / 192.0,
        a75 = -2187.0 / 6784.0, a76 = 11.0 / 84.0;
  constexpr static T bs1 = 5179.0 / 57600.0, bs3 = 7571.0 / 16695.0,
      bs4 = 393.0 / 640.0, bs5 = -92097.0 / 339200.0,
      bs6 = 187.0 / 2100.0, bs7 = 1.0 / 40.0;
  constexpr static T c2 = 1.0 / 5.0, c3 = 3.0 / 10.0, c4 = 4.0 / 5.0,
      c5 = 8.0 / 9.0;

  // Setup scratch buffers
  T *const k1 = bf;
  T *const k2 = k1 + ne;
  T *const k3 = k2 + ne;
  T *const k4 = k3 + ne;
  T *const k5 = k4 + ne;
  T *const k6 = k5 + ne;
  T *const k7 = k6 + ne;
  T *const ks = k7 + ne;
  T *const kz = ks + ne;

  // Initialize local variables
  int err = ODE_ERR_OK;
  unsigned int iter = 0;
  T h_next = (tf - ti) / static_cast<T>(100);
  T t = ti;
  T delta_max;
  T h;

  // Copy yi into yf and ensure we still have some time to integrate over
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i];
  if (ti >= tf) return err | ODE_ERR_BAD_INTERVAL;

  // Calculate the initial k1
  f(t, yf, k1);

  for (;;) {

    // Check iteration bound
    if (iter++ > max_iter) return err | ODE_ERR_MAX_ITER;

    for (;;) {  // Start error checking loop

      // Setup
      h = h_next;

      // Step forward
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * a21 * k1[i];
      f(t + c2 * h, ks, k2);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * (a31 * k1[i] + a32 * k2[i]);
      f(t + c3 * h, ks, k3);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]);
      f(t + c4 * h, ks, k4);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * (a51 * k1[i] + a52 * k2[i] + a53 * k3[i] + a54 * k4[i]);
      f(t + c5 * h, ks, k5);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * (a61 * k1[i] + a62 * k2[i] + a63 * k3[i] + a64 * k4[i] + a65 * k5[i]);
      f(t + h, ks, k6);
      for (unsigned int i = 0; i < ne; i++)
        ks[i] = yf[i] + h * (a71 * k1[i] + a73 * k3[i] + a74 * k4[i] + a75 * k5[i] + a76 * k6[i]);
      f(t + h, ks, k7);
      for (unsigned int i = 0; i < ne; i++)
        kz[i] = yf[i] +
            h * (bs1 * k1[i] + bs3 * k3[i] + bs4 * k4[i] + bs5 * k5[i] + bs6 * k6[i] + bs7 * k7[i]);

      // Calculate largest error
      delta_max = static_cast<T>(0);
      for (unsigned int i = 0; i < ne; i++) {
        T delta = std::abs(ks[i] - kz[i]) /
            std::max(abs_tol, rel_tol * std::max(std::abs(ks[i]), std::abs(kz[i])));
        if (delta > delta_max) delta_max = delta;
      }

      // Helpfull constants
      constexpr static T one = 1.0, five = 5.0, one_fifth = one / five;

      // Extremely small error (prevent divide by zero and bump step size)
      if (delta_max < static_cast<T>(1e-20)) {
        h_next = h * five;
      }
      // Small error (bump step size a little)
      else if (delta_max < one_fifth) {
        h_next = h * std::pow(one / delta_max, one_fifth);
      }
      // Large error (shrink step size, ensure it large enough, and continue)
      else if (delta_max > one) {
        h_next = h * one_fifth;
        if (h_next < h_min) {
          h_next = h_min;
          err |= ODE_ERR_MIN_STEP;
        }
        continue;
      }
      // Step size is working well (don't calculate the next k1 vector)
      else {
        h_next = h;
      }

      // Exit loop to update yf and t
      break;
      
    }  // End error checking loop

    // Step forward time, see if we're done, and prevent to large of a step
    t += h;
    for (unsigned int i = 0; i < ne; i++) yf[i] = ks[i];
    for (unsigned int i = 0; i < ne; i++) k1[i] = k7[i];
    if (t >= tf) return err;
    if (t + h_next >= tf) h_next = tf - t;

  }
}

GNC_ODEXX_TEMPLATE(ode45, float);
GNC_ODEXX_TEMPLATE(ode45, double);

}  // namespace gnc

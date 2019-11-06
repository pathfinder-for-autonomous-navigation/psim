//
// src/gnc_ode.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_ode.hpp>

#include <algorithm>
#include <cmath>

namespace gnc {

template <typename T,
    void (*const S)(T, T, T const *, T *, unsigned int, T *, void (*const)(T, T const *, T *))>
static inline void odex(T const *t, unsigned int nt, T **y, unsigned int ne,
    T *bf, void (*const f)(T, T const *, T *)) {
  // Loop through timesteps
  for (unsigned int i = 1; i < nt; i++)
    S(t[i - 1], t[i] - t[i - 1], y[i - 1], y[i], ne, bf, f);
}

// Reference:
//  https://en.wikipedia.org/wiki/Euler_method
template <typename T>
void ode1(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  // Setup scratch buffers
  T *const k1 = bf;

  // Step forward
  f(ti, yi, k1);
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i] + dt * k1[i];
}

PAN_GNC_ODEX_SINGLE_TEMPLATE(ode1, float);
PAN_GNC_ODEX_SINGLE_TEMPLATE(ode1, double);

template <typename T>
void ode1(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  odex<T, ode1>(t, nt, y, ne, bf, f);
}

PAN_GNC_ODEX_MULTI_TEMPLATE(ode1, float);
PAN_GNC_ODEX_MULTI_TEMPLATE(ode1, double);

// Reference:
//  https://en.wikipedia.org/wiki/List_of_Runge–Kutta_methods#Heun's_method
template <typename T>
void ode2(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  // Table of values
  constexpr static T b1 = 1.0 / 2.0, b2 = 1.0 / 2.0;

  // Setup scratch buffers
  T *const k1 = bf;
  T *const k2 = k1 + ne;
  T *const ks = k2 + ne;

  // Step forward
  f(ti, yi, k1);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + dt * k1[i];
  f(ti + dt, ks, k2);
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i] + dt * (b1 * k1[i] + b2 * k2[i]);
}

PAN_GNC_ODEX_SINGLE_TEMPLATE(ode2, float);
PAN_GNC_ODEX_SINGLE_TEMPLATE(ode2, double);

template <typename T>
void ode2(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  odex<T, ode2>(t, nt, y, ne, bf, f);
}

PAN_GNC_ODEX_MULTI_TEMPLATE(ode2, float);
PAN_GNC_ODEX_MULTI_TEMPLATE(ode2, double);

// Reference:
//  https://en.wikipedia.org/wiki/List_of_Runge–Kutta_methods#Ralston's_method
template <typename T>
void ode3(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  // Table of values
  constexpr static T a21 = 1.0 / 2.0, a22 = 3.0 / 4.0;
  constexpr static T c2 = 1.0 / 2.0, c3 = 3.0 / 4.0;
  constexpr static T b1 = 2.0 / 9.0, b2 = 1.0 / 3.0, b3 = 4.0 / 9.0;

  // Setup scratch buffers
  T *const k1 = bf;
  T *const k2 = k1 + ne;
  T *const k3 = k2 + ne;
  T *const ks = k3 + ne;

  // Step forward
  f(ti, yi, k1);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + a21 * dt * k1[i];
  f(ti + c2 * dt, ks, k2);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + a22 * dt * k2[i];
  f(ti + c3 * dt, ks, k3);
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i] + dt * (b1 * k1[i] + b2 * k2[i] + b3 * k3[i]);
}

PAN_GNC_ODEX_SINGLE_TEMPLATE(ode3, float);
PAN_GNC_ODEX_SINGLE_TEMPLATE(ode3, double);

template <typename T>
void ode3(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  odex<T, ode3>(t, nt, y, ne, bf, f);
}

PAN_GNC_ODEX_MULTI_TEMPLATE(ode3, float);
PAN_GNC_ODEX_MULTI_TEMPLATE(ode3, double);

// Reference:
//  https://en.wikipedia.org/wiki/List_of_Runge–Kutta_methods#Classic_fourth-order_method
template <typename T>
void ode4(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  // Table of values
  constexpr static T a21 = 1.0 / 2.0, a32 = 1.0 / 2.0;
  constexpr static T c2 = 1.0 / 2.0, c3 = 1.0 / 2.0;
  constexpr static T b1 = 1.0 / 6.0, b2 = 1.0 / 3.0, b3 = 1.0 / 3.0, b4 = 1.0 / 6.0;

  // Setup scratch buffers
  T *const k1 = bf;
  T *const k2 = k1 + ne;
  T *const k3 = k2 + ne;
  T *const k4 = k3 + ne;
  T *const ks = k4 + ne;

  // Step forward
  f(ti, yi, k1);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + a21 * dt * k1[i];
  f(ti + c2 * dt, ks, k2);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + a32 * dt * k2[i];
  f(ti + c3 * dt, ks, k3);
  for (unsigned int i = 0; i < ne; i++) ks[i] = yi[i] + dt * k3[i];
  f(ti + dt, ks, k4);
  for (unsigned int i = 0; i < ne; i++)
    yf[i] = yi[i] + dt * (b1 * k1[i] + b2 * k2[i] + b3 * k3[i] + b4 * k4[i]);
}

PAN_GNC_ODEX_SINGLE_TEMPLATE(ode4, float);
PAN_GNC_ODEX_SINGLE_TEMPLATE(ode4, double);

template <typename T>
void ode4(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *)) {
  odex<T, ode4>(t, nt, y, ne, bf, f);
}

PAN_GNC_ODEX_MULTI_TEMPLATE(ode4, float);
PAN_GNC_ODEX_MULTI_TEMPLATE(ode4, double);

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
  bool calc_k1 = true;
  int err = ODE_ERR_OK;
  unsigned int iter = 0;
  T h_next = (tf - ti) / static_cast<T>(10);
  T t = ti;
  T delta_max;
  T h;

  // Copy yi into yf and ensure we still have some time to integrate over
  for (unsigned int i = 0; i < ne; i++) yf[i] = yi[i];
  if (ti >= tf) return err | ODE_ERR_BAD_INTERVAL;

  for (;;) {

    // Check iteration bound
    if (iter++ > max_iter) return err | ODE_ERR_MAX_ITER;

    for (;;) {  // Start error checking loop
    
      // Setup
      h = h_next;
      if (calc_k1) {
        f(t, yf, k1);
        calc_k1 = true;
      }

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
        T delta = abs( ks[i] - kz[i] ) /
            std::max(abs_tol, rel_tol * std::max(abs(ks[i]), abs(kz[i])));
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
        h_next = h * pow(one / delta_max, one_third);
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
        for (unsigned int i = 0; i < ne; i++) k1[i] = k4[i];
        calc_k1 = false;
        h_next = h;
      }

      // Exit loop to update yf and t
      break;

    }  // End errror checking loop

    // Step forward time, see if we're done, and prevent to large of a step
    t += h;
    for (unsigned int i = 0; i < ne; i++) yf[i] = ks[i];
    if (t >= tf) return err;
    if (t + h_next >= tf) h_next = tf - t;
    
  }
}

PAN_GNC_ODEXX_TEMPLATE(ode23, float);
PAN_GNC_ODEXX_TEMPLATE(ode23, double);

}  // namespace gnc

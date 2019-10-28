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

}  // namespace gnc

//
// include/gnc_ode.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/* Fixed and variable step size numerical integrators are declared here.
 *
 * A fixed size integrator will be named odeX where X denotes the order of the
 * integration method. A variable sized method will be named odeXX where XX will
 * denote the two orders of integration being compared to give a sense of accuracy.
 *
 * The fixed step size integrators will have two entrypoints. The first will
 * take a single step given an initial state (ti, yi) and a timestep dt. The new
 * state (ti + dt, yf) will be calculated. The second will take multiple
 * timesteps given a vector of independant variable states t[0], t[1], ...,
 * t[nt]. The initial independant variable conditions y[0] will be used to
 * determine y[1], ..., y[nt]. See the function documentation below for more
 * information.
 */

#ifndef PAN_PSIM_INCLUDE_GNC_ODE_HPP_
#define PAN_PSIM_INCLUDE_GNC_ODE_HPP_

#define PAN_GNC_ODEX_SINGLE_TEMPLATE(odex, type) \
    template \
    void odex<type>(type, type, type const *, type *, unsigned int, type *, \
        void (* const)(type, type const *, type *))

#define PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(odex, type) \
    extern PAN_GNC_ODEX_SINGLE_TEMPLATE(odex, type)

#define PAN_GNC_ODEX_MULTI_TEMPLATE(odex, type) \
    template \
    void odex<type>(type const *, unsigned int, type **, unsigned int, type *, \
        void (* const)(type, type const *, type *))

#define PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(odex, type) \
    extern PAN_GNC_ODEX_MULTI_TEMPLATE(odex, type)

#define PAN_GNC_ODEXX_TEMPLATE(odexx, type) \
    template \
    int odexx<type>(type, type, type const *, type *, unsigned int, type *, \
        type, type, type, unsigned int, void (*const)(type, type const *, type *));

#define PAN_GNC_ODEXX_EXTERN_TEMPLATE(odexx, type) \
    extern PAN_GNC_ODEXX_TEMPLATE(odexx, type)

namespace gnc {

/** @enum ode_err_t
 *  Odexx function error codes. Multiple codes may combined using a bitwise or
 *  operation. */
enum : int {
  /** Zero response code indicate nominal execution (is not a bitmask). */
  ODE_ERR_OK = 0,
  /** A step size below the specified minimum was requested. */
  ODE_ERR_MIN_STEP = (0b1 << 0),
  /** The max number of iterations was reached (integration is incomplete). */
  ODE_ERR_MAX_ITER = (0b1 << 1),
  /** The time interval was ill formatted (integration is incomplete). */
  ODE_ERR_BAD_INTERVAL = (0b1 << 2)
};

/** @fn ode1
 *  @param[in]  ti Initial conditions for the independant variable.
 *  @param[in]  dt Integrator step for the independant variable.
 *  @param[in]  yi Initial conditions for the dependant variables.
 *  @param[out] yf Final state of the system (dependant varaibles).
 *  @param[in]  ne Number of dependant variables.
 *  @param[in]  bf Buffer of length (ne).
 *  @param[in]  f  Dependant variable update function.
 *  Integrates the system from (ti, yi) to (ti + dt, yf) using a first order
 *  method. You may pass yi and yf as the same pointer if yi can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode1(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode1, float);
PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode1, double);

/** @fn ode1
 *  @param[in]     t  Array of independant variables states.
 *  @param[in]     nt Length of the dependant variable array t.
 *  @param[inout]  y  Array of dependant variables states.
 *  @param[in]     ne Number of dependant variables.
 *  @param[in]     bf Buffer of length (ne).
 *  @param[in]     f  Dependant variable update function.
 *  Integrates the system using a first order method through the state sequence
 *  below given t[i] and y[0]:
 *    (t[0], y[0]) -> (t[1], y[1]) -> ... -> (t[nt - 1], y[nt - 1])
 *  Passing y as a list of the same pointer is permissible if intermediate state
 *  information can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode1(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode1, float);
PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode1, double);

/** @fn ode2
 *  @param[in]  ti Initial conditions for the independant variable.
 *  @param[in]  dt Integrator step for the independant variable.
 *  @param[in]  yi Initial conditions for the dependant variables.
 *  @param[out] yf Final state of the system (dependant varaibles).
 *  @param[in]  ne Number of dependant variables.
 *  @param[in]  bf Buffer of length (3 * ne).
 *  @param[in]  f  Dependant variable update function.
 *  Integrates the system from (ti, yi) to (ti + dt, yf) using a second order
 *  method. You may pass yi and yf as the same pointer if yi can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode2(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode2, float);
PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode2, double);

/** @fn ode2
 *  @param[in]     t  Array of independant variables states.
 *  @param[in]     nt Length of the dependant variable array t.
 *  @param[inout]  y  Array of dependant variables states.
 *  @param[in]     ne Number of dependant variables.
 *  @param[in]     bf Buffer of length (3 * ne).
 *  @param[in]     f  Dependant variable update function.
 *  Integrates the system using a second order method through the state sequence
 *  below given t[i] and y[0]:
 *    (t[0], y[0]) -> (t[1], y[1]) -> ... -> (t[nt - 1], y[nt - 1])
 *  Passing y as a list of the same pointer is permissible if intermediate state
 *  information can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode2(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode2, float);
PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode2, double);

/** @fn ode3
 *  @param[in]  ti Initial conditions for the independant variable.
 *  @param[in]  dt Integrator step for the independant variable.
 *  @param[in]  yi Initial conditions for the dependant variables.
 *  @param[out] yf Final state of the system (dependant varaibles).
 *  @param[in]  ne Number of dependant variables.
 *  @param[in]  bf Buffer of length (4 * ne).
 *  @param[in]  f  Dependant variable update function.
 *  Integrates the system from (ti, yi) to (ti + dt, yf) using a third order
 *  method. You may pass yi and yf as the same pointer if yi can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode3(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode3, float);
PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode3, double);

/** @fn ode3
 *  @param[in]     t  Array of independant variables states.
 *  @param[in]     nt Length of the dependant variable array t.
 *  @param[inout]  y  Array of dependant variables states.
 *  @param[in]     ne Number of dependant variables.
 *  @param[in]     bf Buffer of length (4 * ne).
 *  @param[in]     f  Dependant variable update function.
 *  Integrates the system using a third order method through the state sequence
 *  below given t[i] and y[0]:
 *    (t[0], y[0]) -> (t[1], y[1]) -> ... -> (t[nt - 1], y[nt - 1])
 *  Passing y as a list of the same pointer is permissible if intermediate state
 *  information can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode3(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode3, float);
PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode3, double);

/** @fn ode4
 *  @param[in]  ti Initial conditions for the independant variable.
 *  @param[in]  dt Integrator step for the independant variable.
 *  @param[in]  yi Initial conditions for the dependant variables.
 *  @param[out] yf Final state of the system (dependant varaibles).
 *  @param[in]  ne Number of dependant variables.
 *  @param[in]  bf Buffer of length (5 * ne).
 *  @param[in]  f  Dependant variable update function.
 *  Integrates the system from (ti, yi) to (ti + dt, yf) using a fourth order
 *  method. You may pass yi and yf as the same pointer if yi can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode4(T ti, T dt, T const *yi, T *yf, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode4, float);
PAN_GNC_ODEX_SINGLE_EXTERN_TEMPLATE(ode4, double);

/** @fn ode4
 *  @param[in]     t  Array of independant variables states.
 *  @param[in]     nt Length of the dependant variable array t.
 *  @param[inout]  y  Array of dependant variables states.
 *  @param[in]     ne Number of dependant variables.
 *  @param[in]     bf Buffer of length (5 * ne).
 *  @param[in]     f  Dependant variable update function.
 *  Integrates the system using a fourth order method through the state sequence
 *  below given t[i] and y[0]:
 *    (t[0], y[0]) -> (t[1], y[1]) -> ... -> (t[nt - 1], y[nt - 1])
 *  Passing y as a list of the same pointer is permissible if intermediate state
 *  information can be overwritten.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
void ode4(T const *t, unsigned int nt, T **y, unsigned int ne, T *bf,
    void (*const f)(T, T const *, T *));

PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode4, float);
PAN_GNC_ODEX_MULTI_EXTERN_TEMPLATE(ode4, double);

/** @fn ode23
 *  @param[in]  ti       Initial conditions for the independant variable.
 *  @param[in]  tf       Desired final state for the independant variable.
 *  @param[in]  yi       Initial conditions of the dependant variables.
 *  @param[out] yf       Final state of the system (dependant variables).
 *  @param[in]  ne       Number of dependant variables.
 *  @param[in]  bf       Buffer of length (6 * ne).
 *  @param[in]  h_min    Minimum timestep allowed.
 *  @param[in]  rel_tol  Relative tolerance.
 *  @param[in]  abs_tol  Absolute tolerance.
 *  @param[in]  max_iter Maximum number of allowed iterations.
 *  @param[in]  f        Dependant variable update function.
 *  @returns Zero on success (see implementation for more details).
 *  Integrates the system using a variable step size second-third order method
 *  from (ti, yi) -> (tf, yf) where yf is essentially the output. The step size
 *  is bounded from below by h_min.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
int ode23(T ti, T tf, T const *yi, T *yf, unsigned int ne, T *bf, T h_min,
    T rel_tol, T abs_tol, unsigned int max_iter, void (*const f)(T, T const *, T *));

PAN_GNC_ODEXX_EXTERN_TEMPLATE(ode23, float);
PAN_GNC_ODEXX_EXTERN_TEMPLATE(ode23, double);

/** @fn ode45
 *  @param[in]  ti       Initial conditions for the independant variable.
 *  @param[in]  tf       Desired final state for the independant variable.
 *  @param[in]  yi       Initial conditions of the dependant variables.
 *  @param[out] yf       Final state of the system (dependant variables).
 *  @param[in]  ne       Number of dependant variables.
 *  @param[in]  bf       Buffer of length (9 * ne).
 *  @param[in]  h_min    Minimum timestep allowed.
 *  @param[in]  rel_tol  Relative tolerance.
 *  @param[in]  abs_tol  Absolute tolerance.
 *  @param[in]  max_iter Maximum number of allowed iterations.
 *  @param[in]  f        Dependant variable update function.
 *  @returns Zero on success (see implementation for more details).
 *  Integrates the system using a variable step size fourth-fifth order method
 *  from (ti, yi) -> (tf, yf) where yf is essentially the output. The step size
 *  is bounded from below by h_min.
 *  NOTE: Template specializations are provided for double and float types. */
template <typename T>
int ode45(T ti, T tf, T const *yi, T *yf, unsigned int ne, T *bf, T h_min,
    T rel_tol, T abs_tol, unsigned int max_iter, void (*const f)(T, T const *, T *));

PAN_GNC_ODEXX_EXTERN_TEMPLATE(ode45, float);
PAN_GNC_ODEXX_EXTERN_TEMPLATE(ode45, double);

}  // namespace gnc

#endif

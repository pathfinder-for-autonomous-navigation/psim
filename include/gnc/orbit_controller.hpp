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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
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
 */

/** @defgroup orbit_controller Orbit Controller
 *  @brief Defines the interface for the orbit controller.
 * 
 *  More details to come...
 */

#ifndef GNC_ORBIT_CONTROLLER_HPP_
#define GNC_ORBIT_CONTROLLER_HPP_

#include <lin/core.hpp>

namespace gnc {

/** @brief Contains the internal state of an orbit controller.
 * 
 *  The internal state only includes a timestamp of the last recommended firing in
 *  seconds since the PAN epoch. This is to avoid accidently suggesting two
 *  firings due to a single firing node.
 * 
 *  The rest of the state struct serves as a calculation buffer to the controller.
 *  This is to aid with reducing the stack size required to run it.
 *  
 *  @sa OrbitControllerData
 *  @sa OrbitActuation
 * 
 *  @ingroup orbit_controller
 */
struct OrbitControllerState {
  double t_last_firing; //!< Last firing's timestamp since the PAN epoch (s).
  // The below variables serve as a calculation buffer
  lin::Vector3d this_r_ecef0, that_r_ecef0, this_r_hat;
  lin::Vector3d this_v_ecef0, that_v_ecef0, this_v_hat;
  lin::Vector3d this_h_ecef0, that_h_ecef0, this_h_hat;
  lin::Matrix3x3d DCM_hill_ecef0;
  /** @brief Defaults everything's value to NaN. */
  OrbitControllerState();
};

/** @brief Stores the inputs to the control_orbit() function.
 *
 *  These inputs include the time in seconds since the PAN epoch, position in
 *  meters in ECEF, velocity in meters per second in ECEF, relative position in
 *  meters of the other satellite in ECEF, and the relative velocity in meters per
 *  second of the other satellite in ECEF.
 *
 *  All inputs must be finite in order for the orbit controller to make a
 *  successful calculation
 *
 *  @sa OrbitControllerState
 *  @sa OrbitActuation
 * 
 *  @ingroup orbit_controller
 */
struct OrbitControllerData {
  double t;              //!< Time in seconds since the PAN epoch.
  lin::Vector3d r_ecef;  //!< Position in ECEF (m).
  lin::Vector3d v_ecef;  //!< Velocity in ECEF (m/s).
  lin::Vector3d dr_ecef; //!< Relative position of the other satellite in ECEF (m).
  lin::Vector3d dv_ecef; //!< Relative velocity of the other satellite in ECEF (m/s).
  /** @brief Defaults everything's value to NaN. */
  OrbitControllerData();
};

/** @brief Stores outputs of the control_orbit() function.
 * 
 *  These outputs included a recommended impulse vector in Newtons seconds in ECI
 *  and the difference between the next firing nodes phase on the current orbit
 *  and the satellites current position. This allows us to predict when a firing
 *  is about five minutes out.
 * 
 *  @sa OrbitControllerState
 *  @sa OrbitControllerData
 * 
 *  @ingroup orbit_controller
 */
struct OrbitActuation {
  lin::Vector3d J_eci;        //!< Recommended impulse vector (Ns).
  float phase_till_next_node; //!< Phase unil next firing (radians).
  /** @brief Defaults everything's value to NaN. */
  OrbitActuation();
};

#ifdef MEX
void mex_control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation,
    double mass, double K_p, double K_d, double K_e, double K_h);
#else
/** @brief Schedules thruster firings in order to rendezvous.
 * 
 *  @param[in]  state     Calculation buffer.
 *  @param[in]  data      Input data.
 *  @param[out] actuation Updated actuation.
 * 
 *  During most calls to update, the controller will return the phase left until
 *  the next firing node. On these iteration the recommended impulse vector will
 *  be set to NaN.
 * 
 *  On calls to this function when we have reached a firing node and haven't
 *  suggested a firing within a set period of time, the controller will suggest a
 *  firing and populate the impulse vector output. In such a case, the phase to
 *  next firing node will be set to zero as well.
 * 
 *  @ingroup orbit_controller
 */
void control_orbit(struct OrbitControllerState &state,
    struct OrbitControllerData const &data, struct OrbitActuation &actuation);
#endif

}  // namespace gnc

#endif
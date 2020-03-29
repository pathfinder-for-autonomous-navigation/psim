/** @file gnc/attitude_controller.hpp
 *  @author Kyle Krol
 *  Defines the interface for the attitude controllers. */

#ifndef GNC_ATTITUDE_CONTROLLER_HPP_
#define GNC_ATTITUDE_CONTROLLER_HPP_

#include "config.hpp"
#include "containers.hpp"

#include <lin/core.hpp>

namespace gnc {

/** @struct DetumbleControllerState
 *  Contains internal state information needed by the control_detumble function.
 *  */
struct DetumbleControllerState {
  lin::Vector3f mtr_body_cmd;
  CircularBuffer<lin::Vector3f, 10> b_body_buffer;
  /** Defaults everything's value to NaN. */
  DetumbleControllerState();
};

/** @struct DetumbleControllerData
 *  Essentially serves as the inputs to the control_detumble function. See
 *  member documentation for more details. */
struct DetumbleControllerData {
  /** Observed magnetic field in the body frame of the spacecraft. */
  lin::Vector3f b_body;
  /** Defaults everything's value to NaN. */
  DetumbleControllerData();
};

/** @struct DetumbleActuation
 *  Essentially serves as the outputs to the control_detumble function. See
 *  member documentation for more details. */
struct DetumbleActuation {
  /** Magnetourquer actuation command in the body frame (units Am^2). */
  lin::Vector3f mtr_body_cmd;
  /** Defaults everything's value to NaN. */
  DetumbleActuation();
};

/** @fn control_detumble
 *  @param[inout] state     Controller's internal state information.
 *  @param[in]    data      Controller's inputs.
 *  @param[out]   actuation Controller's outputs.
 *  Calculation MTR actuations to detumble the spacecraft. */
void control_detumble(DetumbleControllerState &state,
    DetumbleControllerData const &data, DetumbleActuation &actuation);

/** @struct PointingControllerState
 *  Contains internal state information needed by the control_pointing function.
 *  */
struct PointingControllerState {
  /** Defaults everything's value to NaN. */
  PointingControllerState();
};

/** @struct PointingControllerData
 *  Essentially serves as the inputs to the control_pointing function. Note,
 *  it is permissable to leave some members (like the secondary pointing
 *  objective information) as NaNs. See member documentation for more details.
 *  */
struct PointingControllerData {
  /** Desired state of the primary pointing objective in the body frame. This
   *  needs to be a unit vector. */
  lin::Vector3f primary_desired;
  /** Current state of the primary pointing objective in the body frame. This
   *  needs to be a unit vector. */
  lin::Vector3f primary_current;
  /** Desired state of the secondary pointing objective in the body frame. This
   *  needs to be a unit vector. */
  lin::Vector3f secondary_desired;
  /** Current state of the secondary pointing objective in the body frame. This
   *  needs to be a unit vector. */
  lin::Vector3f secondary_current;
  /** Angular rate of the wheels in the body frame (units of radians/s). */
  lin::Vector3f w_wheels;
  /** Angular rate of the satellite in the body frame (units of radians/s). */
  lin::Vector3f w_sat;
  /** Magnetic field in the body frame (units of T). */
  lin::Vector3f b;
  /** Defaults everything's value to NaN. */
  PointingControllerData();
};

/** @struct PointingActuation
 *  Essentially serves as the outputs to the control_pointing function. See
 *  member documentation for more information. */
struct PointingActuation {
  /** Magnetourquer actuation command in the body frame (units Am^2). */
  lin::Vector3f mtr_body_cmd;
  /** Reaction wheel torque command in the body frame (units Nm). */
  lin::Vector3f rwa_body_cmd;
  /** Defaults everything's value to NaN. */
  PointingActuation();
};

#ifndef MEX
/** @fn control_pointing
 *  @param[inout] state     Controller's internal state information.
 *  @param[in]    data      Controller's inputs.
 *  @param[out]   actuation Controller's outputs.
 *  Calculates wheel and MTR actuations to point to spacecraft according to the
 *  passed objectives. */
void control_pointing(PointingControllerState &state,
    PointingControllerData const &data, PointingActuation &actuation);
#else
/** @fn mex_control_pointing
 *  Mex function entrypoint for MATLAB use. It decouples the pointing controller
 *  from the global controller gains and satellite's moment of inertia value. */
void mex_control_pointing(control_pointing(PointingControllerState &state,
    PointingControllerData const &data, PointingActuation &actuation, float Kp,
    float Kd, lin::Matrix3x3f const &J);
#endif

}  // namespace gnc

#endif

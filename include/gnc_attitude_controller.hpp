//
// include/gnc_attitude_controller.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_ATTITUDE_CONTROLLER_HPP_
#define PAN_PSIM_INCLUDE_GNC_ATTITUDE_CONTROLLER_HPP_

#include "gnc_containers.hpp"
#include "lin.hpp"

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
  /** Magnetourquer actuation command in the body frame (units Am^2) */
  lin::Vector3f mtr_body_cmd;
  /** Defaults everything's value to NaN. */
  DetumbleActuation();
};

/** @fn control_detumble
 *  @brief Calculation MTR actuations to detumble the spacecraft.
 *  @param[inout] state     Control function internal state information.
 *  @param[in]    data      Control function inputs
 *  @param[out]   actuation Actuation output. */
void control_detumble(DetumbleControllerState &state,
    DetumbleControllerData const &data, DetumbleActuation &actuation);

}  // namespace gnc

#endif

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

/** @file psim/core/model.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_SIMULATION_HPP_
#define PSIM_CORE_SIMULATION_HPP_

#include <psim/core/model.hpp>
#include <psim/core/state.hpp>

namespace psim {

/** @brief Represents a full simulation.
 *
 *  @tparam C Underlying model used by the simulation.
 *
 *  The main purpose of the simulation class is to merge the functionality of a
 *  state and model. It's how end users are intended to interact with models,
 *  state fields, and steps.
 */
template <class C>
class Simulation : public State {
 private:
  /** @brief Random number generator shared across the entire simulation.
   */
  RandomsGenerator _randoms;

  /** @brief Model employed by the simulation.
   */
  C _model;

 public:
  Simulation() = delete;
  Simulation(Simulation const &) = delete;
  Simulation(Simulation &&) = delete;
  Simulation &operator=(Simulation const &) = delete;
  Simulation &operator=(Simulation &&) = delete;

  virtual ~Simulation() = default;

  /** @brief Creates a simulation based on the provided configuration.
   *
   *  @param[in] config Simulation configuration.
   *
   *  Note, the simulation expects a field named 'seed' in the configuration to
   *  initialize the random number generator.
   */
  Simulation(Configuration const &config)
    : _randoms(config["seed"].get<Integer>()), _model(_randoms, config) {
    _model.add_fields(*this);
    _model.get_fields(*this);
  }

  /** @brief Steps the simulation (and all underlying models) forward.
   */
  void step() {
    _model.step();
  }
};
} // namespace psim

#endif

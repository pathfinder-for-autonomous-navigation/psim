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

#include <memory>
#include <utility>

namespace psim {

/** @brief Represents a full simulation.
 *
 *  The main purpose of the simulation class is to merge the functionality of a
 *  state and model. It's how end users are intended to interact with models,
 *  state fields, and steps.
 */
class Simulation : public State {
 private:
  /** @brief Model employed by the simulation.
   *
   *  Making this field dynamically allocated was very intentional. It ensures
   *  on moves that the stored state fields address remain valid.
   */
  std::unique_ptr<Model> _model;

 protected:
  /** @brief Private constructor employed by the factory function.
   */
  Simulation(std::unique_ptr<Model> &&model);

 public:
  Simulation() = delete;
  Simulation(Simulation const &) = delete;
  Simulation(Simulation &&) = default;
  Simulation &operator=(Simulation const &) = delete;
  Simulation &operator=(Simulation &&) = default;

  virtual ~Simulation() = default;

  /** @brief Steps the simulation (and all underlying models) forward.
   */
  void step();

  /** @brief Create a simulation based on the given model type.
   *
   *  @tparam C  Model type.
   *  @tparam Ts Model constructor arguments.
   *
   *  @return A simulation.
   */
  template <class C, typename... Ts>
  static Simulation make(Ts &&... ts) {
    return Simulation(std::make_unique<C>(std::forward<Ts>(ts)...));
  }
};
} // namespace psim

#endif

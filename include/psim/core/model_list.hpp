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

/** @file psim/core/model_list.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_MODEL_LIST_HPP_
#define PSIM_CORE_MODEL_LIST_HPP_

#include <psim/core/model.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace psim {

/** @brief A model consisting of multiple models run in series.
 */
class ModelList : public Model {
 private:
  /** @brief Model list.
   */
  std::vector<std::unique_ptr<Model>> _models;

 protected:
  ModelList(RandomsGenerator &randoms);

  /** @brief Adds a new model to the model list.
   *
   *  @tparam C  Model type.
   *  @tparam Ts Model constructor argument types.
   *
   *  @param[in] ts Model constructor arguments.
   *
   *  Note that the models are executed in the order they are added to the model
   *  list.
   */
  template <class C, typename... Ts>
  void add(Ts &&... ts) {
    _models.push_back(std::make_unique<C>(std::forward<Ts>(ts)...));
  }

 public:
  virtual ~ModelList() = default;

  /** @brief All models add their state fields to the simulation state.
   *
   *  @param[in] state Simulation state.
   */
  virtual void add_fields(State &state) override;

  /** @brief All models request extra fields they need from the simulation
   *  state.
   *
   *  @param[in] state Simulation state.
   */
  virtual void get_fields(State &state) override;

  /** @brief All models step forward.
   */
  virtual void step() override;
};
} // namespace psim

#endif

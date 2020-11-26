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

#ifndef PSIM_CORE_MODEL_HPP_
#define PSIM_CORE_MODEL_HPP_

#include <psim/core/configuration.hpp>
#include <psim/core/state.hpp>
#include <psim/core/state_field.hpp>

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace psim {

/** @brief The base unit of "logic" in a simulation.
 *
 *  Every simulation relies on a model type to declare state fields, request
 *  state fields, and contain all the neccesary functionality to step the
 *  simulation forward in time.
 */
class Model {
 protected:
  /** @brief Reference to the simulation's random number generator.
   */
  RandomsGenerator &_randoms;

  /** @brief Sets the model random number generator.
   *
   *  @param[in] randoms
   */
  Model(lin::internal::RandomsGenerator &randoms);

  /** @brief Retrive a field from the simulation state.
   *
   *  @tparam T Underlying type.
   *
   *  @param[in] state Simulation state.
   *  @param[in] name  Field name.
   *
   *  @return Pointer to the state field.
   *
   *  Note, if the field wasn't found or has an invalid underlying type, a
   * runtime error will be thrown.
   */
  template <typename T>
  StateField<T> const *get_field(State const &state, std::string const &name) {
    auto const *field_ptr =
        dynamic_cast<StateField<T> const *>(state.get(name));
    if (!field_ptr)
      throw std::runtime_error("Invalid cast while getting a field: " + name);

    return field_ptr;
  }

  /** @brief Retrive a writable field from the simulation state.
   *
   *  @tparam T Underlying type.
   *
   *  @param[in] state Simulation state.
   *  @param[in] name  Field name.
   *
   *  @return Pointer to the writable state field.
   *
   *  Note, if the field wasn't found or has an invalid underlying type, a
   * runtime error will be thrown.
   */
  template <typename T>
  StateFieldWritable<T> *get_writable_field(
      State &state, std::string const &name) {
    auto *field_ptr =
        dynamic_cast<StateFieldWritable<T> *>(state.get_writable(name));
    if (!field_ptr)
      throw std::runtime_error(
          "Invalid cast while getting a writable field: " + name);

    return field_ptr;
  }

 public:
  Model() = delete;
  Model(Model const &) = delete;
  Model(Model &&) = delete;
  Model &operator=(Model const &) = delete;
  Model &operator=(Model &&) = delete;

  virtual ~Model() = default;

  /** @brief The model adds it's state fields to the simulation state.
   *
   *  @param[in] state Simulation state.
   */
  virtual void add_fields(State &state);

  /** @brief The model requests extra fields it needs from the simulation state.
   *
   *  @param[in] state Simulation state.
   */
  virtual void get_fields(State &state);

  /** @brief The model steps forward.
   *
   *  This essentially is the update step that is responsible for updating state
   *  fields values.
   */
  virtual void step();
};
} // namespace psim

#endif

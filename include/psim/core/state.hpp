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

/** @file psim/core/state.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_HPP_
#define PSIM_CORE_STATE_HPP_

#include <psim/core/state_field.hpp>

#include <functional>
#include <string>
#include <unordered_map>

namespace psim {

/** @brief Collection of state fields comprising a simulation's state.
 *
 *  This class only holds pointers to the actually fields that are held by the
 *  models themselves. Therefore, the state is invalid if any models are
 *  deallocated.
 *
 *  The fields are mapped one to one with a string name. It's not possible to have
 *  a readable and writable field registered to the same name.
 */
class State {
 private:
  /** @brief Map to the readable fields.
   */
  std::unordered_map<std::reference_wrapper<std::string const>, StateFieldBase const *,
      std::hash<std::string>, std::equal_to<std::string>> _readable_fields;

  /** @brief Map to the writable fields.
   */
  std::unordered_map<std::reference_wrapper<std::string const>, StateFieldWritableBase *,
      std::hash<std::string>, std::equal_to<std::string>> _writable_fields;

 public:
  State() = default;
  State(State const &) = delete;
  State(State &&) = default;
  State &operator=(State const &) = delete;
  State &operator=(State &&) = default;

  virtual ~State() = default;

  /** @param[in] name Field name.
   *
   *  @return True if a field is registered under the given name and false
   *          otherwise.
   */
  bool has(std::string const &name) const;

  /** @param[in] name Field name.
   *
   *  @return True if a writable field is registered under the given name and
   *          false otherwise.
   */
  bool has_writable(std::string const &name) const;

  /** @brief Add a new field to the simulation state.
   *
   *  @param[in] name Field name.
   *  @param[in] field_ptr Pointer to the new field.
   *
   *  If a field already exists under the given name, a runtime error will be
   *  thrown.
   */
  void add(StateFieldBase const *field_ptr);

  /** @brief Add a new writable field to the simulation state.
   *
   *  @param[in] name Field name.
   *  @param[in] field_ptr Pointer to the new writable field.
   *
   *  If a field already exists under the given name, a runtime error will be
   *  thrown.
   */
  void add_writable(StateFieldWritableBase *field_ptr);

  /** @brief Retrieve a field from the simulation state.
   *
   *  @param[in] name Field name.
   *
   *  @return Pointer to the retrieved field.
   *
   *  If no such field exists, a null pointer is returned.
   */
  StateFieldBase const *get(std::string const &name) const;

  /** @brief Retrieve a writable field from the simulation state.
   *
   *  @param[in] name Field name.
   *
   *  @return Pointer to the retrieved writable field.
   *
   *  If no such field exists, a null pointer is returned.
   */
  StateFieldWritableBase *get_writable(std::string const &name);

  /** @brief Retrieve a field from the simulation state.
   *
   *  @param[in] name Field name.
   *
   *  @return Reference to the retrieved field.
   *
   *  If no such field exists, a runtime error will be thrown.
   */
  StateFieldBase const &operator[](std::string const &name) const;
};
}  // namespace psim

#endif

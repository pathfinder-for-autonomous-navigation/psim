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

/** @file psim/core/state.cpp
 *  @author Kyle Krol
 */

#include <psim/core/state.hpp>

#include <stdexcept>

namespace psim {

bool State::has(std::string const &name) const {
  return (has_writable(name) || _readable_fields.count(name));
}

bool State::has_writable(std::string const &name) const {
  return _writable_fields.count(name);
}

void State::add_writable(StateFieldWritableBase *field) {
  {
    // Check if we have a name collision with existing readable fields
    auto const iter = _readable_fields.find(field->name());
    if (iter != _readable_fields.end()) {
      auto const *_field = iter->second;
      throw std::runtime_error(
          "'" + _field->name() + ":" + _field->type() +
          "' already registered " + "as a readable field. Cannot add '" +
          field->name() + ":" + field->type() + "' as a writable field");
    }
  }
  {
    // Check if we have a name collision with existing writable fields
    auto const iter = _writable_fields.find(field->name());
    if (iter != _writable_fields.end()) {
      auto *_field = iter->second;
      throw std::runtime_error(
          "'" + _field->name() + ":" + _field->type() +
          "' already registered " + "as a writable field. Cannot add '" +
          field->name() + ":" + field->type() + "' as a writable field");
    }
  }

  // Add the field
  _writable_fields[field->name()] = field;
}

void State::add(StateFieldBase const *field) {
  {
    // Check if we have a name collision with existing readable fields
    auto const iter = _readable_fields.find(field->name());
    if (iter != _readable_fields.end()) {
      auto const *_field = iter->second;
      throw std::runtime_error(
          "'" + _field->name() + ":" + _field->type() +
          "' already registered " + "as a readable field. Cannot add '" +
          field->name() + ":" + field->type() + "' as a readable field");
    }
  }
  {
    // Check if we have a name collision with existing writable fields
    auto const iter = _writable_fields.find(field->name());
    if (iter != _writable_fields.end()) {
      auto *_field = iter->second;
      throw std::runtime_error(
          "'" + _field->name() + ":" + _field->type() +
          "' already registered " + "as a writable field. Cannot add '" +
          field->name() + ":" + field->type() + "' as a readable field");
    }
  }

  // Add the field
  _readable_fields[field->name()] = field;
}

StateFieldWritableBase *State::get_writable(std::string const &name) {
  auto const iter = _writable_fields.find(name);
  return (iter != _writable_fields.end() ? iter->second : nullptr);
}

StateFieldBase const *State::get(std::string const &name) const {
  {
    auto const iter = _readable_fields.find(name);
    if (iter != _readable_fields.end())
      return iter->second;
  }
  {
    auto const iter = _writable_fields.find(name);
    if (iter != _writable_fields.end())
      return iter->second;
  }

  return nullptr;
}

StateFieldBase const &State::operator[](std::string const &name) const {
  auto const *field_ptr = get(name);
  if (!field_ptr)
    throw std::runtime_error("State field not found with name: " + name);

  return *field_ptr;
}
} // namespace psim

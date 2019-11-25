//
// include/gnc_containers.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_CONTAINERS_HPP_
#define PAN_PSIM_INCLUDE_GNC_CONTAINERS_HPP_

#include <array>
#include <type_traits>

namespace gnc {

/** @class CircularBuffer
 *  @tparam T Data type stored in the buffer.
 *  @tparam N Maximum number of elements that can be stored in the buffer.
 *  Simple circular buffer implementation that requires the data type to have a
 *  default constructor. */
template <typename T, std::size_t N>
class CircularBuffer {
 public:
  constexpr CircularBuffer();
  constexpr CircularBuffer(CircularBuffer<T, N> const &) = default;
  constexpr CircularBuffer(CircularBuffer<T, N> &&) = default;
  constexpr CircularBuffer<T, N> &operator=(CircularBuffer<T, N> const &) = default;
  constexpr CircularBuffer<T, N> &operator=(CircularBuffer<T, N> &&) = default;
  /** @returns True if the buffer contains the maximum number of elements. */
  constexpr bool full() const;
  /** @returns True if the buffer contains no elements. */
  constexpr bool empty() const;
  /** @returns Number of elements currently in the buffer. */
  constexpr std::size_t size() const;
  /** @returns Maximum number of elements that can be stored in the buffer. */
  constexpr std::size_t max_size() const;
  /** @returns Read only reference to the i'th element in the buffer. */
  constexpr T const &operator[](std::size_t i) const;
  /** @returns Read, write access to the i'th element in the buffer. */
  constexpr T &operator[](std::size_t i);
  /** Adds an element to the buffer. */
  constexpr void push(T const &t);
  /** Removes all elements from the buffer. */
  constexpr void clear();

 private:
  T array[N];     //< Backing array.
  std::size_t s;  //< Start position counter
  std::size_t n;  //<
};
}  // namespace gnc

#include "inl/gnc_containers.inl"

#endif

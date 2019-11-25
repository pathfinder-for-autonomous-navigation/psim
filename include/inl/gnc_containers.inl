//
// include/inl/gnc_containers.inl
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "../gnc_containers.hpp"

namespace gnc {

template <typename T, std::size_t N>
constexpr CircularBuffer<T, N>::CircularBuffer()
: array(), s(0), n(0) { }

template <typename T, std::size_t N>
constexpr bool CircularBuffer<T, N>::full() const {
  return size() == max_size();
}

template <typename T, std::size_t N>
constexpr bool CircularBuffer<T, N>::empty() const {
  return size() == 0;
}

template <typename T, std::size_t N>
constexpr std::size_t CircularBuffer<T, N>::size() const {
  return n;
}

template <typename T, std::size_t N>
constexpr std::size_t CircularBuffer<T, N>::max_size() const {
  return N;
}

template <typename T, std::size_t N>
constexpr T const &CircularBuffer<T, N>::operator[](std::size_t i) const {
  return array[(s + i) % max_size()];
}

template <typename T, std::size_t N>
constexpr T &CircularBuffer<T, N>::operator[](std::size_t i) {
  return array[(s + i) % max_size()];
}

template <typename T, std::size_t N>
constexpr void CircularBuffer<T, N>::push(T const &t) {
  // Copy the element into the circular buffer
  array[(s + size()) % max_size()] = t;
  // Adjust the start and size counters
  if (!full()) n++;
  else s = (s + 1) % max_size();
}

template <typename T, std::size_t N>
constexpr void CircularBuffer<T, N>::clear() {
  n = 0;
}
}  // namespace gnc

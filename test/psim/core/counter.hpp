/** @file test/psim/core/counter.hpp
 *  @author Kyle Krol
 */

#ifndef TEST_PSIM_CORE_COUNTER_HPP_
#define TEST_PSIM_CORE_COUNTER_HPP_

#include <psim/core/configuration.hpp>
#include <psim/core/model.hpp>
#include <psim/core/parameter.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>

/** @brief Simple model that counts an integer forward for testing purposes.
 */
class Counter : public psim::Model {
 private:
  psim::StateFieldValued<psim::Integer> _dn;
  psim::StateFieldValued<psim::Integer> _n;

 public:
  Counter() = delete;
  virtual ~Counter() = default;

  Counter(psim::RandomsGenerator &randoms, psim::Configuration const &config);
  virtual void add_fields(psim::State &state) override;
  virtual void step() override;
};

#endif

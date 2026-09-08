#include <cassert>
#include <cstdio>

#include "sg200x_clock_tree.hpp"

namespace Tree = LibXR::SG200XClockTree;
using Tree::ClockId;
using Tree::RatePolicy;

static_assert(Tree::IsTopologyWellFormed());
static_assert(Tree::ArePeripheralResourcesWellFormed());
static_assert(Tree::IsParentValid(ClockId::Spi, ClockId::Fpll));
static_assert(!Tree::IsParentValid(ClockId::Spi, ClockId::Mpll));
static_assert(Tree::IsDividerValid(ClockId::Spi, 63u));
static_assert(!Tree::IsDividerValid(ClockId::Spi, 64u));
static_assert(!Tree::IsFieldWellFormed(Tree::Field(0x014u, 0u)));
static_assert(!Tree::IsFieldWellFormed(Tree::Field(CLKGEN_CLK_EN0_OFFSET, 31u, 2u)));
static_assert(Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, 1500000000u, 187500000u)
                  .divider == 8u);

int main()
{
  // Volatile input intentionally proves that the public planner is callable at runtime.
  volatile uint32_t rate_input = 187500000u;
  volatile uint32_t parent_input = 1500000000u;
  auto plan = Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, parent_input, rate_input);
  assert(plan.IsValid() && plan.actual_rate_hz == 187500000u && plan.divider == 8u);
  assert(Tree::IsClockPlanWellFormed(plan));
  plan.divider = 7u;
  assert(!Tree::IsClockPlanWellFormed(plan));

  plan = Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, parent_input, 50000000u);
  assert(plan && plan.divider == 30u && plan.actual_rate_hz == 50000000u);
  assert(!Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, parent_input, 20000000u));
  assert(!Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, parent_input, 0u));
  assert(!Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, 0u, 100000000u));
  assert(!Tree::MakeRatePlan(ClockId::Spi, ClockId::Mpll, parent_input, 100000000u));
  assert(!Tree::MakeRatePlan(ClockId::Fpll, ClockId::Oscillator, 25000000u, 1500000000u));
  assert(!Tree::MakeRatePlan(ClockId::C906_1, ClockId::Fpll, parent_input, 500000000u));
  assert(!Tree::MakeRatePlan(static_cast<ClockId>(0x7777u), ClockId::Fpll, parent_input,
                             100000000u));

  // The oscillator input to SPI bypasses its divider. Clock1M does not bypass it.
  plan = Tree::MakeRatePlan(ClockId::Spi, ClockId::Oscillator, 25000000u, 25000000u);
  assert(plan && plan.divider == 1u && plan.parent_index == 0u);
  assert(!Tree::MakeRatePlan(ClockId::Spi, ClockId::Oscillator, 25000000u, 12500000u));
  plan = Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 1000000u);
  assert(plan && plan.divider == 25u);

  plan = Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 3000000u);
  assert(plan && plan.divider == 8u && plan.actual_rate_hz == 3125000u);
  plan = Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 3000000u,
                            RatePolicy::AtMost);
  assert(plan && plan.divider == 9u && plan.actual_rate_hz == 2777777u);
  assert(!Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 3000000u,
                             RatePolicy::Exact));
  // Integer truncation must not turn 25 MHz / 7 into an "exact" or at-most 3,571,428 Hz.
  assert(!Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 3571428u,
                             RatePolicy::Exact));
  plan = Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 3571428u,
                            RatePolicy::AtMost);
  assert(plan && plan.divider == 8u);
  plan = Tree::MakeRatePlan(ClockId::Clock1M, ClockId::Oscillator, 25000000u, 5625000u);
  assert(plan && plan.divider == 5u);  // Equal error: choose the lower output rate.

  plan = Tree::MakeRatePlan(ClockId::Pwm, ClockId::Fpll, parent_input, 100000000u);
  assert(plan && plan.clock == ClockId::Pwm && plan.rate_clock == ClockId::PwmSource);
  assert(Tree::Find(Tree::PeripheralId::Sdio0)->rate_clock == ClockId::Sd0);
  assert(Tree::Find(Tree::PeripheralId::Spi3)->rate_clock == ClockId::Spi);
  std::puts("Runtime planner and static topology/field/resource checks passed");
}

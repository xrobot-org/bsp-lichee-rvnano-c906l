#include <sys/mman.h>

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>

#include "sg200x_rcc.hpp"
#include "sg200x_ll.h"

namespace Tree = LibXR::SG200XClockTree;
using ClockId = Tree::ClockId;
using PeripheralId = Tree::PeripheralId;
using LibXR::ErrorCode;

static void Map(uintptr_t address)
{
  assert(mmap(reinterpret_cast<void*>(address), 4096u, PROT_READ | PROT_WRITE,
              MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1,
              0) == reinterpret_cast<void*>(address));
}

static std::array<uint8_t, 8192u> Snapshot()
{
  std::array<uint8_t, 8192u> memory{};
  std::memcpy(memory.data(), CLKGEN, 4096u);
  std::memcpy(memory.data() + 4096u, RSTGEN, 4096u);
  return memory;
}

static uint32_t Fpll(uint32_t multiplier)
{
  return (1u << PLL_G6_PREDIV_SHIFT) | (1u << PLL_G6_POSTDIV_SHIFT) |
         (multiplier << PLL_G6_MULTIPLIER_SHIFT);
}

int main()
{
  Map(CLKGEN_BASE);
  Map(RSTGEN_BASE);
  auto& clock = LibXR::SG200XRCC::Instance();
  CLKGEN->CLK_BYP0 = (1u << 15u) | (1u << 19u) | (1u << 20u) | (1u << 30u) | (1u << 31u);
  CLKGEN->CLK_EN[0] = 0x80000000u;  // Unrelated fields must survive every RMW.
  CLKGEN->CLK_EN[3] = 0x80000000u;
  PLL_G6->FPLL = Fpll(56u);
  const auto defaults =
      Tree::MakeRatePlan(ClockId::Spi, ClockId::Fpll, 1500000000u, 187500000u);
  auto before = Snapshot();
  assert(clock.ApplyClockPlan(defaults) == ErrorCode::CHECK_ERR);
  assert(clock.PreparePeripheral(PeripheralId::Spi2) == ErrorCode::CHECK_ERR);
  assert(before == Snapshot());

  PLL_G6->FPLL = Fpll(60u);
  // Fail on a later resource after AXI4 and its SD gate have already been written.
  // The unknown G2 parent must roll the earlier writes and initialized flags back.
  CLKGEN->DIV_SD0 = (1u << 8u) | (15u << 16u) | 9u;
  before = Snapshot();
  assert(clock.PreparePeripheral(PeripheralId::Sdio0) == ErrorCode::NOT_SUPPORT);
  assert(before == Snapshot());
  CLKGEN->DIV_SD0 = 0u;
  assert(clock.PreparePeripheral(PeripheralId::Spi2) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 187500000u);
  assert(clock.ClockRate(ClockId::Axi4) == 300000000u);
  assert(clock.IsClockEnabled(ClockId::Spi) && clock.IsClockEnabled(ClockId::ApbSpi2));
  assert((RSTGEN->SOFT_RSTN[1] & (1u << 10u)) != 0u);
  assert(CLKGEN->CLK_EN[0] == 0x80000000u && (CLKGEN->CLK_EN[3] & 0x80000000u) != 0u);

  before = Snapshot();
  auto plan = clock.PlanRate(ClockId::Spi, 100000000u);
  assert(plan && plan.divider == 15u && plan.parent == ClockId::Fpll);
  assert(before == Snapshot());
  assert(clock.ApplyClockPlan(plan) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 100000000u &&
         clock.IsClockEnabled(ClockId::Spi));
  assert(clock.EnableClock(ClockId::Spi) == ErrorCode::OK);
  assert(clock.PreparePeripheral(PeripheralId::Spi1) == ErrorCode::OK);
  assert(clock.ResetPeripheral(PeripheralId::Spi2) == ErrorCode::OK);
  assert(clock.ClockRate(PeripheralId::Spi0) == 100000000u);
  assert(clock.ClockRate(PeripheralId::Spi3) == 100000000u);
  assert(clock.DisableClock(ClockId::Spi) == ErrorCode::OK);
  assert(!clock.IsClockEnabled(ClockId::Spi) &&
         clock.ClockRate(ClockId::Spi) == 100000000u);
  assert(clock.EnableClock(ClockId::Spi) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 100000000u);

  before = Snapshot();
  auto forged = plan;
  forged.actual_rate_hz = 1u;
  assert(clock.ApplyClockPlan(forged) == ErrorCode::ARG_ERR);
  assert(clock.SetRate(ClockId::Spi, 1u) == ErrorCode::NOT_SUPPORT);
  assert(clock.SetRate(ClockId::Fpll, 1000000000u) == ErrorCode::NOT_SUPPORT);
  assert(clock.SetRate(ClockId::C906_1, 500000000u) == ErrorCode::NOT_SUPPORT);
  assert(clock.SetRate(ClockId::Axi4, 150000000u) == ErrorCode::NOT_SUPPORT);
  assert(clock.DisableClock(ClockId::Axi4) == ErrorCode::NOT_SUPPORT);
  assert(before == Snapshot());
  PLL_G6->FPLL = Fpll(56u);
  before = Snapshot();
  assert(clock.ApplyClockPlan(plan) == ErrorCode::CHECK_ERR);
  assert(before == Snapshot());
  PLL_G6->FPLL = Fpll(60u);

  assert(clock.SetRate(PeripheralId::Spi2, 50000000u) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 50000000u);
  const auto bypass =
      Tree::MakeRatePlan(ClockId::Spi, ClockId::Oscillator, 25000000u, 25000000u);
  assert(clock.ApplyClockPlan(bypass) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 25000000u);
  assert(clock.EnableClock(ClockId::Spi) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Spi) == 25000000u);

  assert(clock.PreparePeripheral(PeripheralId::I2c0) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Axi6) == 100000000u &&
         clock.ClockRate(ClockId::I2c) == 100000000u);
  assert(clock.SetRate(ClockId::I2c, 50000000u, Tree::RatePolicy::Exact) ==
         ErrorCode::OK);
  assert(clock.PreparePeripheral(PeripheralId::I2c1) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::I2c) == 50000000u);

  assert(clock.SetRate(ClockId::Pwm, 100000000u) == ErrorCode::OK);
  assert(clock.EnableClock(ClockId::Pwm) == ErrorCode::OK);
  before = Snapshot();
  assert(clock.DisableClock(ClockId::PwmSource) == ErrorCode::BUSY);
  assert(clock.SetRate(ClockId::Pwm, 50000000u) == ErrorCode::BUSY);
  assert(before == Snapshot());
  assert(clock.DisableClock(ClockId::Pwm) == ErrorCode::OK);
  assert(clock.SetRate(PeripheralId::Pwm2, 50000000u) == ErrorCode::OK);
  assert(clock.EnableClock(ClockId::Pwm) == ErrorCode::OK);
  assert(clock.ClockRate(ClockId::Pwm) == 50000000u);
  assert(PLL_G6->FPLL == Fpll(60u));
  std::puts(
      "Runtime clock control, default preservation, bypass, dependencies and rejection "
      "checks passed");
}

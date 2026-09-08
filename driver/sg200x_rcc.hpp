#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "libxr_def.hpp"
#include "sg200x_clock_tree.hpp"

namespace LibXR
{

/** Runtime controller for the statically validated SG2002 clock graph.
 * Calls that change hardware are serialized without masking interrupts.
 * Rate changes require quiescent consumers; SPI/I2C/PWM roots are shared.
 * PLLs and CPU clocks remain firmware-owned. Shared system buses may only
 * receive the agreed startup profile while inactive, never an arbitrary runtime rate. */
class SG200XRCC final
{
 public:
  static_assert(std::atomic<uint32_t>::is_always_lock_free);
  using ClockId = SG200XClockTree::ClockId;
  using PeripheralId = SG200XClockTree::PeripheralId;
  using ResetId = SG200XClockTree::ResetId;
  using ClockRatePlan = SG200XClockTree::ClockRatePlan;
  using RatePolicy = SG200XClockTree::RatePolicy;

  enum class SpiController : uint8_t
  {
    SPI0,
    SPI1,
    SPI2,
    SPI3
  };
  enum class I2cController : uint8_t
  {
    I2C0,
    I2C1,
    I2C2,
    I2C3,
    I2C4
  };

  [[nodiscard]] static SG200XRCC& Instance() noexcept;

  /** Plan against current parent rates without writing hardware. */
  [[nodiscard]] ClockRatePlan PlanRate(
      ClockId clock, uint32_t target_rate_hz,
      RatePolicy policy = RatePolicy::Nearest) const noexcept;
  [[nodiscard]] ClockRatePlan PlanRate(
      PeripheralId peripheral, uint32_t target_rate_hz,
      RatePolicy policy = RatePolicy::Nearest) const noexcept;

  /** Validate a plan and its parent rate, enable parents, and program the branch.
   * Preserve its prior gate state. Stale/invalid plans cause no writes; a failed
   * write is rolled back. Disable enabled child clocks before retiming their parent. */
  [[nodiscard]] ErrorCode ApplyClockPlan(const ClockRatePlan& plan) noexcept;
  [[nodiscard]] ErrorCode SetRate(ClockId clock, uint32_t target_rate_hz,
                                  RatePolicy policy = RatePolicy::Nearest) noexcept;
  [[nodiscard]] ErrorCode SetRate(PeripheralId peripheral, uint32_t target_rate_hz,
                                  RatePolicy policy = RatePolicy::Nearest) noexcept;

  /** Enable the selected path. Startup defaults are applied only once; explicit
   * runtime configuration is retained across EnableClock/PreparePeripheral. */
  [[nodiscard]] ErrorCode EnableClock(ClockId clock) noexcept;
  /** Disable only this gate; enabled descendants and system/CPU clocks are protected. */
  [[nodiscard]] ErrorCode DisableClock(ClockId clock) noexcept;
  [[nodiscard]] bool IsClockEnabled(ClockId clock) const noexcept;

  [[nodiscard]] ErrorCode ReleaseReset(ResetId reset) noexcept;
  [[nodiscard]] ErrorCode PreparePeripheral(PeripheralId peripheral) noexcept;
  [[nodiscard]] ErrorCode ResetPeripheral(PeripheralId peripheral) noexcept;

  /** Nominal configured rate, including when gated off. Zero means unknown or
   * a concurrent update. G2 fractional PLL rates are not guessed. */
  [[nodiscard]] uint32_t ClockRate(ClockId clock) const noexcept;
  [[nodiscard]] uint32_t ClockRate(PeripheralId peripheral) const noexcept;

 private:
  struct Transaction;
  SG200XRCC() = default;
  [[nodiscard]] bool TryBeginWrite() noexcept;
  void EndWrite() noexcept;
  [[nodiscard]] ClockRatePlan PlanRateLocked(ClockId clock, uint32_t target_rate_hz,
                                             RatePolicy policy) const noexcept;
  [[nodiscard]] ErrorCode ApplyClockPlanLocked(const ClockRatePlan& plan,
                                               Transaction& transaction,
                                               bool startup) noexcept;
  [[nodiscard]] ErrorCode InitializeClockLocked(ClockId clock, Transaction& transaction,
                                                uint8_t depth) noexcept;
  [[nodiscard]] ErrorCode EnableClockPathLocked(ClockId clock, Transaction& transaction,
                                                uint8_t depth, bool defaults) noexcept;
  [[nodiscard]] ErrorCode ReleaseResetLocked(ResetId reset) noexcept;
  [[nodiscard]] ErrorCode PulseResetLocked(ResetId reset) noexcept;
  [[nodiscard]] bool PlanMatchesHardware(const ClockRatePlan& plan) const noexcept;
  [[nodiscard]] bool PathContains(ClockId clock, ClockId ancestor,
                                  uint8_t depth) const noexcept;
  [[nodiscard]] bool HasEnabledDependent(ClockId clock) const noexcept;
  [[nodiscard]] bool IsClockEnabledRecursive(ClockId clock, uint8_t depth) const noexcept;
  void RememberPath(ClockId clock, Transaction& transaction, uint8_t depth) noexcept;
  [[nodiscard]] uint32_t ClockRateRecursive(ClockId clock, uint8_t depth) const noexcept;

  std::atomic_flag write_busy_ = ATOMIC_FLAG_INIT;
  std::atomic<uint32_t> write_sequence_{0u};
  std::array<bool, SG200XClockTree::NODES.size()> configured_{};
  static SG200XRCC instance_;
};

}  // namespace LibXR

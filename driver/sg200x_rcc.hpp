#pragma once

#include <atomic>
#include <cstdint>

#include "libxr_def.hpp"
#include "sg200x_clock_tree.hpp"

namespace LibXR
{

/**
 * @brief Runtime provider for the SG200x TOP clock/reset tree.
 *
 * `SG200XClockTree` describes the hardware topology and C906L clock plan at
 * compile time. This singleton validates the board's FPLL contract, applies
 * the needed C906L mux/divider route, enables its gates, and releases its
 * peripheral reset line. PLL roots and the running C906L core clock remain
 * read-only at runtime.
 */
class SG200XRCC final
{
 public:
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "SG200XRCC requires lock-free 32-bit atomics");

  using ClockId = SG200XClockTree::ClockId;
  using PeripheralId = SG200XClockTree::PeripheralId;
  using ResetId = SG200XClockTree::ResetId;

  enum class SpiController : uint8_t
  {
    SPI0,
    SPI1,
    SPI2,
    SPI3,
  };

  enum class I2cController : uint8_t
  {
    I2C0,
    I2C1,
    I2C2,
    I2C3,
    I2C4,
  };

  [[nodiscard]] static SG200XRCC& Instance() noexcept;

  /**
   * Enable a modelled C906L clock path and apply its compile-time rate plan.
   * Returns BUSY instead of waiting when another clock/reset update is active.
   * This control-plane operation must not be called from an ISR.
   */
  [[nodiscard]] ErrorCode EnableClock(ClockId clock) noexcept;

  /**
   * Release one active-low reset line from the board's reset-controller ABI.
   * Returns BUSY instead of waiting and must not be called from an ISR.
   */
  [[nodiscard]] ErrorCode ReleaseReset(ResetId reset) noexcept;

  /**
   * Enable every clock gate listed for an actual peripheral and release its
   * reset. This is idempotent; it does not assert/reset an active client.
   * Returns BUSY instead of waiting and must not be called from an ISR.
   */
  [[nodiscard]] ErrorCode PreparePeripheral(PeripheralId peripheral) noexcept;

  /**
   * Enable a peripheral's clocks, assert its active-low reset, then release
   * it. The caller must ensure that no other client is using the peripheral.
   * Returns BUSY instead of waiting and must not be called from an ISR.
   */
  [[nodiscard]] ErrorCode ResetPeripheral(PeripheralId peripheral) noexcept;

  /**
   * Decode the live hardware rate. A zero result means invalid hardware state,
   * an unmodelled fractional PLL path, or overlap with a clock/reset update;
   * callers must not substitute a guessed rate. This method never waits.
   */
  [[nodiscard]] uint32_t ClockRate(ClockId clock) const noexcept;

 private:
  SG200XRCC() = default;

  [[nodiscard]] bool TryBeginWrite() noexcept;
  void EndWrite() noexcept;
  [[nodiscard]] ErrorCode EnableClockPathLocked(ClockId clock, uint8_t depth) noexcept;
  [[nodiscard]] ErrorCode ApplyC906LClockPlanLocked(ClockId clock,
                                                     uint8_t depth) noexcept;
  [[nodiscard]] ErrorCode ReleaseResetLocked(ResetId reset) noexcept;
  [[nodiscard]] ErrorCode PulseResetLocked(ResetId reset) noexcept;
  [[nodiscard]] uint32_t ClockRateRecursive(ClockId clock, uint8_t depth) const noexcept;

  std::atomic_flag write_busy_ = ATOMIC_FLAG_INIT;
  std::atomic<uint32_t> write_sequence_{0u};
  static SG200XRCC instance_;
};

}  // namespace LibXR

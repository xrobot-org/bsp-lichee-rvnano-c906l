#pragma once

#include <cstdint>

#include "timebase.hpp"

namespace LibXR
{

/**
 * @brief SG200x C906 CSR-based LibXR timebase using rdtime instruction.
 *
 * LibXR exposes timestamp entry points as static backend hooks, so their
 * definitions intentionally remain `Timebase::GetMicroseconds()` and
 * `Timebase::GetMilliseconds()` rather than C++ virtual overrides.
 */
class SG200XTimebase : public Timebase
{
 public:
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 25000000u;

  /**
   * @param clock_hz time CSR frequency in Hz. This must match the
   *                 `timebase-frequency` value supplied to the firmware.
   */
  explicit SG200XTimebase(uint32_t clock_hz = DEFAULT_CLOCK_HZ);

  /** @brief Return the configured timer input frequency. */
  [[nodiscard]] static uint32_t ClockHz() noexcept;
};

}  // namespace LibXR

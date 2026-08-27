#pragma once
#include <atomic>
#include <cstdint>
#include "watchdog.hpp"
namespace LibXR
{
class SG200XWatchdog final : public Watchdog
{
 public:
  enum class Instance : uint8_t { WDT0 = 0u, WDT1, WDT2 };
  enum class ResetTarget : uint8_t { CPU = 0u, SYSTEM };
  enum class ResponseMode : uint8_t { RESET = 0u, INTERRUPT_THEN_RESET };
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 25000000u;
  static constexpr uint32_t XTAL32K_HZ = 32768u;
  static constexpr uint32_t C906L_WDT_IRQ = 39u;
  static constexpr uint32_t TOP_SYS_CTRL_OFFSET = 0x008u;
  explicit SG200XWatchdog(Instance instance = Instance::WDT2, uint32_t timeout_ms = 1000u,
                          uint32_t feed_ms = 250u, uint32_t clock_hz = DEFAULT_CLOCK_HZ,
                          ResetTarget reset_target = ResetTarget::CPU,
                          ResponseMode response_mode = ResponseMode::RESET);
  ErrorCode SetConfig(const Configuration& config) override;
  ErrorCode Feed() override;
  ErrorCode Start() override;
  ErrorCode Stop() override;
  [[nodiscard]] bool IsValid() const noexcept { return base_ != 0u; }
  [[nodiscard]] bool IsRunning() const noexcept { return running_; }
  [[nodiscard]] uint32_t ActualTimeoutMs() const noexcept { return actual_timeout_ms_; }
 [[nodiscard]] uint8_t TimeoutTop() const noexcept { return timeout_top_; }
 private:
  static constexpr uintptr_t WDT0_BASE = 0x03010000u;
  static constexpr uintptr_t WDT1_BASE = 0x03011000u;
  static constexpr uintptr_t WDT2_BASE = 0x03012000u;
  static constexpr uintptr_t TOP_BASE = 0x03000000u;
  static constexpr uint32_t REG_CR = 0x00u;
  static constexpr uint32_t REG_TORR = 0x04u;
  static constexpr uint32_t REG_CRR = 0x0Cu;
  static constexpr uint32_t REG_TOC = 0x1Cu;
  static constexpr uint32_t REG_TOP_WDT_CTRL = 0x1A8u;
  static constexpr uint32_t CR_ENABLE = 1u;
  static constexpr uint32_t RESTART_KEY = 0x76u;
  static constexpr uint8_t MAX_TOP = 15u;
  static uintptr_t InstanceBase(Instance instance) noexcept;
  static bool IsSupportedClock(uint32_t clock_hz) noexcept;
  static bool IsSupportedResetTarget(ResetTarget target) noexcept;
  static bool IsSupportedResponseMode(ResponseMode mode) noexcept;
  static uint32_t PeriodMs(uint32_t clock_hz, uint8_t top) noexcept;
  bool TryLock() noexcept;
  void Unlock() noexcept;
  ErrorCode SelectTimeout(uint32_t timeout_ms);
  void ConfigureResetRoute() const noexcept;
  uintptr_t base_ = 0u;
  uint32_t reset_route_bit_ = 0u;
  uint32_t clock_hz_ = 0u;
  ResetTarget reset_target_ = ResetTarget::CPU;
  ResponseMode response_mode_ = ResponseMode::RESET;
  uint32_t actual_timeout_ms_ = 0u;
  uint8_t timeout_top_ = 0u;
  bool running_ = false;
  std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
};
}  // namespace LibXR

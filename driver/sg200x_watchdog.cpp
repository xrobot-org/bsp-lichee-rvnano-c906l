#include "sg200x_watchdog.hpp"
#include "sg200x_mmio.hpp"
namespace LibXR
{
uintptr_t SG200XWatchdog::InstanceBase(Instance instance) noexcept
{
  switch (instance)
  {
    case Instance::WDT0:
      return WDT0_BASE;
    case Instance::WDT1:
      return WDT1_BASE;
    case Instance::WDT2:
      return WDT2_BASE;
  }
  return 0u;
}
bool SG200XWatchdog::IsSupportedClock(uint32_t clock_hz) noexcept
{
  return clock_hz == DEFAULT_CLOCK_HZ || clock_hz == XTAL32K_HZ;
}
bool SG200XWatchdog::IsSupportedResetTarget(ResetTarget target) noexcept
{
  return target == ResetTarget::CPU || target == ResetTarget::SYSTEM;
}
bool SG200XWatchdog::IsSupportedResponseMode(ResponseMode mode) noexcept
{
  return mode == ResponseMode::RESET || mode == ResponseMode::INTERRUPT_THEN_RESET;
}
uint32_t SG200XWatchdog::PeriodMs(uint32_t clock_hz, uint8_t top) noexcept
{
  const uint64_t cycles = 1ULL << (16u + top);
  return static_cast<uint32_t>((cycles * 1000ULL + clock_hz - 1u) / clock_hz);
}
SG200XWatchdog::SG200XWatchdog(Instance instance, uint32_t timeout_ms, uint32_t feed_ms,
                               uint32_t clock_hz, ResetTarget reset_target,
                               ResponseMode response_mode)
    : base_(InstanceBase(instance)),
      reset_route_bit_(static_cast<uint32_t>(instance) <= 2u
                           ? 1u << static_cast<uint32_t>(instance)
                           : 0u),
      clock_hz_(clock_hz),
      reset_target_(reset_target),
      response_mode_(response_mode)
{
  if (!IsValid() || !IsSupportedClock(clock_hz_) ||
      !IsSupportedResetTarget(reset_target_) || !IsSupportedResponseMode(response_mode_) ||
      SetConfig({timeout_ms, feed_ms}) != ErrorCode::OK || Start() != ErrorCode::OK)
  {
    base_ = 0u;
  }
}
bool SG200XWatchdog::TryLock() noexcept
{
  return !lock_.test_and_set(std::memory_order_acquire);
}
void SG200XWatchdog::Unlock() noexcept { lock_.clear(std::memory_order_release); }
ErrorCode SG200XWatchdog::SelectTimeout(uint32_t timeout_ms)
{
  if (timeout_ms == 0u)
  {
    return ErrorCode::ARG_ERR;
  }
  for (uint8_t top = 0u; top <= MAX_TOP; ++top)
  {
    const uint32_t period_ms = PeriodMs(clock_hz_, top);
    if (period_ms >= timeout_ms)
    {
      timeout_top_ = top;
      actual_timeout_ms_ = period_ms;
      return ErrorCode::OK;
    }
  }
  return ErrorCode::NOT_SUPPORT;
}
ErrorCode SG200XWatchdog::SetConfig(const Configuration& config)
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  if (!IsSupportedClock(clock_hz_))
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (config.timeout_ms == 0u || config.feed_ms == 0u ||
      config.feed_ms > config.timeout_ms)
  {
    return ErrorCode::ARG_ERR;
  }
  if (running_)
  {
    return ErrorCode::BUSY;
  }
  if (!TryLock())
  {
    return ErrorCode::BUSY;
  }
  const ErrorCode result = SelectTimeout(config.timeout_ms);
  if (result == ErrorCode::OK)
  {
    timeout_ms_ = actual_timeout_ms_;
    auto_feed_interval_ms = config.feed_ms;
    Register32(base_, REG_TORR) = static_cast<uint32_t>(timeout_top_) |
                                  (static_cast<uint32_t>(timeout_top_) << 4u);
    Register32(base_, REG_TOC) = 0u;
    Register32(base_, REG_CR) = 0u;
  }
  Unlock();
  return result;
}
void SG200XWatchdog::ConfigureResetRoute() const noexcept
{
  // Enable the watchdog reset request, then select either the C906L CPU or the
  // whole system for this watchdog instance.
  Register32(TOP_BASE, TOP_SYS_CTRL_OFFSET) |= 1u << 2u;
  auto& top = Register32(TOP_BASE, REG_TOP_WDT_CTRL);
  const uint32_t system_bit = reset_route_bit_;
  const uint32_t cpu_bit = reset_route_bit_ << 4u;
  if (reset_target_ == ResetTarget::CPU)
  {
    top = (top & ~system_bit) | cpu_bit;
  }
  else
  {
    top = (top & ~cpu_bit) | system_bit;
  }
  top = (top & ~(0x7u << 8u)) | (clock_hz_ == XTAL32K_HZ ? (1u << 8u) : 0u);
}
ErrorCode SG200XWatchdog::Feed()
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  Register32(base_, REG_CRR) = RESTART_KEY;
  return ErrorCode::OK;
}
ErrorCode SG200XWatchdog::Start()
{
  if (!IsValid() || running_ || !TryLock())
  {
    return IsValid() ? ErrorCode::BUSY : ErrorCode::ARG_ERR;
  }
  ConfigureResetRoute();
  Register32(base_, REG_TORR) = static_cast<uint32_t>(timeout_top_) |
                                (static_cast<uint32_t>(timeout_top_) << 4u);
  Register32(base_, REG_CRR) = RESTART_KEY;
  Register32(base_, REG_CR) =
      CR_ENABLE | (response_mode_ == ResponseMode::INTERRUPT_THEN_RESET ? (1u << 1u) : 0u);
  running_ = true;
  auto_feed_ = true;
  Unlock();
  return ErrorCode::OK;
}
ErrorCode SG200XWatchdog::Stop()
{
  auto_feed_ = false;
  return ErrorCode::NOT_SUPPORT;
}
}  // namespace LibXR

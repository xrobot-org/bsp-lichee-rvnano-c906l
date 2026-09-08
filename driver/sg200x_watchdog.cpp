#include "sg200x_watchdog.hpp"

#include "sg200x_ll_rcc.h"
#include "sg200x_rcc.hpp"
namespace LibXR
{
WDT_Type* SG200XWatchdog::InstanceRegisters(Instance instance) noexcept
{
  return sgll_wdt_get(static_cast<uint32_t>(instance));
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
  const uint64_t cycles = sgll_wdt_timeout_cycles(top);
  return static_cast<uint32_t>((cycles * 1000ULL + clock_hz - 1u) / clock_hz);
}
SG200XWatchdog::SG200XWatchdog(Instance instance, uint32_t timeout_ms, uint32_t feed_ms,
                               uint32_t clock_hz, ResetTarget reset_target,
                               ResponseMode response_mode)
    : regs_(InstanceRegisters(instance)),
      instance_index_(static_cast<uint8_t>(instance)),
      clock_hz_(clock_hz),
      reset_target_(reset_target),
      response_mode_(response_mode)
{
  if (!IsValid() || !IsSupportedClock(clock_hz_) ||
      !IsSupportedResetTarget(reset_target_) ||
      !IsSupportedResponseMode(response_mode_) ||
      SG200XRCC::Instance().PreparePeripheral(SG200XRCC::PeripheralId::Watchdog) !=
          ErrorCode::OK ||
      SetConfig({timeout_ms, feed_ms}) != ErrorCode::OK || Start() != ErrorCode::OK)
  {
    regs_ = nullptr;
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
  for (uint8_t top = 0u; top <= WDT_TOP_MAX; ++top)
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
  ErrorCode result = SelectTimeout(config.timeout_ms);
  if (result == ErrorCode::OK)
  {
    const sgll_wdt_init_t init = {.top = timeout_top_, .interrupt_first = false};
    if (sgll_wdt_init(regs_, &init))
    {
      timeout_ms_ = actual_timeout_ms_;
      auto_feed_interval_ms = config.feed_ms;
    }
    else
    {
      result = ErrorCode::BUSY;
    }
  }
  Unlock();
  return result;
}
void SG200XWatchdog::ConfigureResetRoute() const noexcept
{
  (void)sgll_wdt_reset_route_set(instance_index_, reset_target_ == ResetTarget::CPU);
  sgll_rcc_watchdog_clock_select(clock_hz_ == XTAL32K_HZ);
}
ErrorCode SG200XWatchdog::Feed()
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  sgll_wdt_feed(regs_);
  return ErrorCode::OK;
}
ErrorCode SG200XWatchdog::Start()
{
  if (!IsValid() || running_ || !TryLock())
  {
    return IsValid() ? ErrorCode::BUSY : ErrorCode::ARG_ERR;
  }
  ConfigureResetRoute();
  if (!sgll_wdt_start(regs_, timeout_top_,
                      response_mode_ == ResponseMode::INTERRUPT_THEN_RESET))
  {
    Unlock();
    return ErrorCode::BUSY;
  }
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

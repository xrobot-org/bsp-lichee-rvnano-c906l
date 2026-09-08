#include "sg200x_pwm.hpp"

#include <cmath>

#include "sg200x_ll_pinmux.h"
#include "sg200x_rcc.hpp"

namespace LibXR
{
namespace
{

uint32_t ClampDutyTicks(uint64_t ticks, uint32_t period) noexcept
{
  if (ticks < 1u)
  {
    return 1u;
  }
  if (ticks >= period)
  {
    return period - 1u;
  }
  return static_cast<uint32_t>(ticks);
}
}  // namespace

SG200XPWM::SG200XPWM(Channel channel, uint32_t clock_hz)
    : SG200XPWM(channel, clock_hz, PinmuxConfiguration{INVALID_PINMUX, 0u})
{
}

SG200XPWM::SG200XPWM(Channel channel, uint32_t clock_hz, PinmuxConfiguration pinmux)
    : SG200XPWM(static_cast<uint8_t>(channel), clock_hz, pinmux)
{
}

SG200XPWM::SG200XPWM(uint8_t channel, uint32_t clock_hz)
    : SG200XPWM(channel, clock_hz, PinmuxConfiguration{INVALID_PINMUX, 0u})
{
}

SG200XPWM::SG200XPWM(uint8_t channel, uint32_t clock_hz, PinmuxConfiguration pinmux)
    : channel_(channel), clock_hz_(clock_hz), pinmux_(pinmux)
{
  if (channel_ >= CHANNEL_COUNT || clock_hz_ == 0u)
  {
    return;
  }
  const auto peripheral = static_cast<SG200XRCC::PeripheralId>(
      static_cast<uint8_t>(SG200XRCC::PeripheralId::Pwm0) +
      channel_ / PWM_CHANNELS_PER_CONTROLLER);
  if (SG200XRCC::Instance().PreparePeripheral(peripheral) != ErrorCode::OK)
  {
    return;
  }

  regs_ = ControllerInstance(channel_);
  channel_mask_ = static_cast<uint32_t>(1u) << LocalChannel(channel_);
  ConfigurePinmux(pinmux_);
}

uint8_t SG200XPWM::LocalChannel(uint8_t channel) noexcept
{
  return static_cast<uint8_t>(channel % PWM_CHANNELS_PER_CONTROLLER);
}

PWM_Type* SG200XPWM::ControllerInstance(uint8_t channel) noexcept
{
  return channel < CHANNEL_COUNT ? sgll_pwm_get(channel / PWM_CHANNELS_PER_CONTROLLER)
                                 : nullptr;
}

void SG200XPWM::ConfigurePinmux(const PinmuxConfiguration& pinmux) noexcept
{
  (void)sgll_pinmux_function_set(pinmux.offset, pinmux.function);
}

uint32_t SG200XPWM::DutyToHighTicks(float duty, uint32_t period) noexcept
{
  if (!std::isfinite(duty) || duty <= 0.0f)
  {
    return 1u;
  }
  if (duty >= 1.0f)
  {
    return period - 1u;
  }

  const auto ticks = static_cast<uint64_t>(
      std::lround(static_cast<double>(duty) * static_cast<double>(period)));
  return ClampDutyTicks(ticks, period);
}

void SG200XPWM::WritePeriodRegisters() noexcept
{
  (void)sgll_pwm_period_set(regs_, LocalChannel(channel_), period_ticks_,
                            period_ticks_ - high_ticks_);
}

void SG200XPWM::ApplyDynamicUpdate() noexcept
{
  (void)sgll_pwm_update(regs_, channel_mask_);
}

ErrorCode SG200XPWM::SetConfig(Configuration config)
{
  if (!IsValid() || config.frequency == 0u)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint64_t period =
      (static_cast<uint64_t>(clock_hz_) + config.frequency / 2u) / config.frequency;
  if (period < 2u || period > MAX_PERIOD_TICKS)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  period_ticks_ = static_cast<uint32_t>(period);
  high_ticks_ = DutyToHighTicks(duty_cycle_, period_ticks_);
  WritePeriodRegisters();

  if (enabled_)
  {
    ApplyDynamicUpdate();
  }
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::SetDutyCycle(float value)
{
  if (!IsValid() || period_ticks_ < 2u || !std::isfinite(value))
  {
    return ErrorCode::ARG_ERR;
  }

  if (value < 0.0f)
  {
    value = 0.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  duty_cycle_ = value;
  high_ticks_ = DutyToHighTicks(value, period_ticks_);
  WritePeriodRegisters();
  if (enabled_)
  {
    ApplyDynamicUpdate();
  }
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::SetPolarity(bool active_high)
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  if (enabled_)
  {
    return ErrorCode::BUSY;
  }

  active_high_ = active_high;
  sgll_pwm_polarity_set(regs_, channel_mask_, active_high_);
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::Enable()
{
  if (!IsValid() || period_ticks_ < 2u)
  {
    return ErrorCode::STATE_ERR;
  }

  (void)sgll_pwm_start(regs_, channel_mask_, active_high_);
  enabled_ = true;
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::Disable()
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }

  (void)sgll_pwm_stop(regs_, channel_mask_);
  enabled_ = false;
  return ErrorCode::OK;
}

}  // namespace LibXR

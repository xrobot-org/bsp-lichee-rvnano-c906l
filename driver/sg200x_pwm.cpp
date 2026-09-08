#include "sg200x_pwm.hpp"

#include <cmath>

#include "sg200x_mmio.hpp"
#include "sg200x_rcc.hpp"

namespace LibXR
{
namespace
{
constexpr uint8_t CHANNEL_PER_CONTROLLER = 4u;

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
      channel_ / CHANNEL_PER_CONTROLLER);
  if (SG200XRCC::Instance().PreparePeripheral(peripheral) != ErrorCode::OK)
  {
    return;
  }

  base_ = ControllerBase(channel_);
  channel_mask_ = static_cast<uint32_t>(1u) << LocalChannel(channel_);
  ConfigurePinmux(pinmux_);
}

uint8_t SG200XPWM::LocalChannel(uint8_t channel) noexcept
{
  return static_cast<uint8_t>(channel % CHANNEL_PER_CONTROLLER);
}

uintptr_t SG200XPWM::ControllerBase(uint8_t channel) noexcept
{
  if (channel >= CHANNEL_COUNT)
  {
    return 0u;
  }
  return PWM0_BASE +
         (static_cast<uintptr_t>(channel / CHANNEL_PER_CONTROLLER) * CONTROLLER_STRIDE);
}

void SG200XPWM::ConfigurePinmux(const PinmuxConfiguration& pinmux) noexcept
{
  if (pinmux.offset == INVALID_PINMUX || pinmux.function > 7u)
  {
    return;
  }

  auto& reg = Register32(PINMUX_BASE, pinmux.offset);
  reg = (reg & ~PINMUX_FUNCTION_MASK) |
        (static_cast<uint32_t>(pinmux.function) & PINMUX_FUNCTION_MASK);
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
  const uintptr_t channel_base =
      base_ + static_cast<uintptr_t>(LocalChannel(channel_)) * CHANNEL_STRIDE;
  Register32(base_, channel_base - base_ + REG_HLPERIOD) = period_ticks_ - high_ticks_;
  Register32(base_, channel_base - base_ + REG_PERIOD) = period_ticks_;
}

void SG200XPWM::ApplyDynamicUpdate() noexcept
{
  auto& update = Register32(base_, REG_PWMUPDATE);
  update |= channel_mask_;
  update &= ~channel_mask_;
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
  auto& polarity = Register32(base_, REG_POLARITY);
  polarity = active_high_ ? polarity & ~channel_mask_ : polarity | channel_mask_;
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::Enable()
{
  if (!IsValid() || period_ticks_ < 2u)
  {
    return ErrorCode::STATE_ERR;
  }

  auto& polarity = Register32(base_, REG_POLARITY);
  polarity = active_high_ ? polarity & ~channel_mask_ : polarity | channel_mask_;
  auto& start = Register32(base_, REG_PWMSTART);
  start &= ~channel_mask_;
  auto& output_enable = Register32(base_, REG_PWM_OE);
  output_enable |= channel_mask_;
  start |= channel_mask_;
  enabled_ = true;
  return ErrorCode::OK;
}

ErrorCode SG200XPWM::Disable()
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }

  Register32(base_, REG_PWM_OE) &= ~channel_mask_;
  Register32(base_, REG_PWMSTART) &= ~channel_mask_;
  enabled_ = false;
  return ErrorCode::OK;
}

}  // namespace LibXR

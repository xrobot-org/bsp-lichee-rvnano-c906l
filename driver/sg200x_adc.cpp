#include "sg200x_adc.hpp"

#include <cmath>

#include "sg200x_mmio.hpp"

namespace LibXR
{

float SG200XADC::Channel::Read()
{
  return parent_ == nullptr ? -1.0f : parent_->ReadChannel(index_);
}

SG200XADC::SG200XADC(std::initializer_list<uint8_t> channels)
    : SG200XADC(channels, Config{})
{
}

SG200XADC::SG200XADC(std::initializer_list<uint8_t> channels, const Config& config)
    : reference_voltage_(config.reference_voltage),
      reference_(config.reference),
      clock_divider_(config.clock_divider),
      timeout_iterations_(config.timeout_iterations)
{
  if (!std::isfinite(reference_voltage_) || reference_voltage_ <= 0.0f ||
      clock_divider_ > 15u || timeout_iterations_ == 0u ||
      channels.size() > CHANNEL_COUNT)
  {
    return;
  }

  for (const uint8_t channel : channels)
  {
    if (channel < 1u || channel > CHANNEL_COUNT)
    {
      channel_count_ = 0u;
      return;
    }
    for (uint8_t index = 0u; index < channel_count_; ++index)
    {
      if (channel_numbers_[index] == channel)
      {
        channel_count_ = 0u;
        return;
      }
    }
    channel_numbers_[channel_count_] = channel;
    channels_[channel_count_] = Channel(this, channel_count_, channel);
    ++channel_count_;
  }

  if (channel_count_ == 0u)
  {
    return;
  }

  EnableClocksAndReset();
  ConfigureDomain(ACTIVE_BASE);
  ConfigureDomain(RTC_BASE);
  valid_ = true;
}

SG200XADC::Channel& SG200XADC::GetChannel(uint8_t index) noexcept
{
  // Keep the reference-returning LibXR driver convention without asserting in
  // firmware: invalid callers receive an inert reader whose Read() is -1 V.
  static Channel invalid;
  return index < channel_count_ ? channels_[index] : invalid;
}

float SG200XADC::ReadChannel(uint8_t index) noexcept
{
  uint16_t raw = 0u;
  if (ReadRaw(index, raw) != ErrorCode::OK)
  {
    return -1.0f;
  }
  return static_cast<float>(raw) * reference_voltage_ / static_cast<float>(MAX_RAW);
}

ErrorCode SG200XADC::ReadRaw(uint8_t index, uint16_t& value) noexcept
{
  if (!valid_ || index >= channel_count_)
  {
    return ErrorCode::ARG_ERR;
  }

  bool expected = false;
  if (!conversion_active_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed))
  {
    return ErrorCode::BUSY;
  }

  const uint8_t logical_channel = channel_numbers_[index];
  const uintptr_t base = DomainBase(logical_channel);
  const uint8_t domain_channel = DomainChannel(logical_channel);
  const uint32_t result_offset =
      REG_RESULT_BASE + static_cast<uint32_t>(domain_channel - 1u) * sizeof(uint32_t);
  ErrorCode result = WaitIdle(base);
  if (result == ErrorCode::OK)
  {
    uint32_t control = Register32(base, REG_CTRL);
    control &= ~(CHANNEL_SELECT_MASK | TRIGGER);
    // The TRM exposes a one-hot select in bits 7:4. This is also the encoding
    // used by the Linux cvitek SARADC driver: channel 1 -> bit 5, etc.
    control |= 1u << (4u + domain_channel);
    Register32(base, REG_CTRL) = control;
    Register32(base, REG_INTR_CLR) = 1u;
    Register32(base, REG_CTRL) = control | TRIGGER;

    result = ErrorCode::TIMEOUT;
    for (uint32_t attempt = 0u; attempt < timeout_iterations_; ++attempt)
    {
      if ((Register32(base, REG_STATUS) & STATUS_BUSY) != 0u)
      {
        continue;
      }

      const uint32_t sample = Register32(base, result_offset);
      if ((sample & RESULT_VALID) != 0u)
      {
        value = static_cast<uint16_t>(sample & MAX_RAW);
        result = ErrorCode::OK;
      }
      else
      {
        result = ErrorCode::FAILED;
      }
      break;
    }
  }

  conversion_active_.store(false, std::memory_order_release);
  return result;
}

uintptr_t SG200XADC::DomainBase(uint8_t channel) noexcept
{
  return channel <= 3u ? ACTIVE_BASE : RTC_BASE;
}

uint8_t SG200XADC::DomainChannel(uint8_t channel) noexcept
{
  return channel <= 3u ? channel : static_cast<uint8_t>(channel - 3u);
}

void SG200XADC::EnableClocksAndReset() noexcept
{
  // The active SARADC clock is gated in CLKGEN. RTC SARADC uses the RTC
  // domain's XTAL-selected clock and its own reset control.
  Register32(CLOCK_GEN_BASE, 0x000u) |= CLOCK_SARADC;
  Register32(RTC_CTRL_BASE, RTC_REG_CLOCK_MUX) &= ~RTC_SARADC_CLOCK_MUX;
  Register32(RTC_CTRL_BASE, RTC_REG_RESET) |= RTC_SARADC_RESETN;
}

void SG200XADC::ConfigureDomain(uintptr_t base) const noexcept
{
  uint32_t cycles = Register32(base, REG_CYC_SET);
  cycles &=
      ~(CYCLE_SETTLE_MASK | CYCLE_SAMPLE_MASK | CYCLE_DIVIDER_MASK | CYCLE_COMPARE_MASK);
  cycles |= CYCLE_SETTLE_DEFAULT;
  cycles |= CYCLE_SAMPLE_DEFAULT;
  cycles |= static_cast<uint32_t>(clock_divider_) << 12u;
  cycles |= CYCLE_COMPARE_DEFAULT;
  Register32(base, REG_CYC_SET) = cycles;
  Register32(base, REG_INTR_EN) = 0u;
  Register32(base, REG_INTR_CLR) = 1u;

  uint32_t test = Register32(base, REG_TEST);
  test = (test & ~TEST_REFERENCE_MASK) |
         (static_cast<uint32_t>(reference_) << TEST_REFERENCE_SHIFT);
  Register32(base, REG_TEST) = test;
}

ErrorCode SG200XADC::WaitIdle(uintptr_t base) const noexcept
{
  for (uint32_t attempt = 0u; attempt < timeout_iterations_; ++attempt)
  {
    if ((Register32(base, REG_STATUS) & STATUS_BUSY) == 0u)
    {
      return ErrorCode::OK;
    }
  }
  return ErrorCode::TIMEOUT;
}

}  // namespace LibXR

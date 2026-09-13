#include "sg200x_adc.hpp"

#include <cmath>

#include "sg200x_ll_rcc.h"
#include "sg200x_rcc.hpp"

namespace LibXR
{

float SG200XADC::Channel::Read()
{
  return parent_ == nullptr ? -1.0f : parent_->ReadChannel(index_);
}

SG200XADC::SG200XADC(std::initializer_list<uint8_t> channels, Config config)
    : reference_voltage_(config.reference_voltage),
      reference_(config.reference),
      clock_divider_(config.clock_divider),
      timeout_iterations_(config.timeout_iterations)
{
  if (!std::isfinite(reference_voltage_) || reference_voltage_ <= 0.0f ||
      clock_divider_ > SARADC_CYCLE_DIVIDER_MAX || timeout_iterations_ == 0u ||
      channels.size() > CHANNEL_COUNT ||
      (reference_ != Reference::INTERNAL && reference_ != Reference::EXTERNAL_VDD18A))
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

  if (EnableClocksAndReset() != ErrorCode::OK)
  {
    channel_count_ = 0u;
    return;
  }
  if (ConfigureDomain(sg200x_ll_adc_get(0u)) != ErrorCode::OK ||
      ConfigureDomain(sg200x_ll_adc_get(1u)) != ErrorCode::OK)
  {
    channel_count_ = 0u;
    return;
  }
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
  SARADC_Type* const adc = DomainInstance(logical_channel);
  const uint8_t domain_channel = DomainChannel(logical_channel);
  ErrorCode result = WaitIdle(adc);
  if (result == ErrorCode::OK)
  {
    if (!sg200x_ll_adc_start(adc, domain_channel))
    {
      result = ErrorCode::BUSY;
    }
    else
    {
      result = WaitIdle(adc);
      if (result == ErrorCode::OK && !sg200x_ll_adc_result_read(adc, domain_channel, &value))
      {
        result = ErrorCode::FAILED;
      }
    }
  }

  conversion_active_.store(false, std::memory_order_release);
  return result;
}

SARADC_Type* SG200XADC::DomainInstance(uint8_t channel) noexcept
{
  return sg200x_ll_adc_get(channel <= SARADC_CHANNEL_COUNT ? 0u : 1u);
}

uint8_t SG200XADC::DomainChannel(uint8_t channel) noexcept
{
  return channel <= SARADC_CHANNEL_COUNT
             ? channel
             : static_cast<uint8_t>(channel - SARADC_CHANNEL_COUNT);
}

ErrorCode SG200XADC::EnableClocksAndReset() noexcept
{
  // The active-domain gate/reset belongs to the shared TOP resource model.
  // SG200X LL also prepares the independent RTC-domain clock/reset path.
  const ErrorCode result =
      SG200XRCC::Instance().PreparePeripheral(SG200XRCC::PeripheralId::SarAdc);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  sg200x_ll_rcc_rtc_saradc_enable();
  return ErrorCode::OK;
}

ErrorCode SG200XADC::ConfigureDomain(SARADC_Type* adc) const noexcept
{
  const ErrorCode idle = WaitIdle(adc);
  if (idle != ErrorCode::OK) return idle;
  sg200x_ll_adc_init_t config;
  sg200x_ll_adc_struct_init(&config);
  config.clock_divider = clock_divider_;
  config.external_reference = reference_ == Reference::EXTERNAL_VDD18A;
  return sg200x_ll_adc_init(adc, &config) ? ErrorCode::OK : ErrorCode::STATE_ERR;
}

ErrorCode SG200XADC::WaitIdle(const SARADC_Type* adc) const noexcept
{
  return sg200x_ll_adc_wait_idle(adc, timeout_iterations_) ? ErrorCode::OK
                                                      : ErrorCode::TIMEOUT;
}

}  // namespace LibXR

#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "adc.hpp"
#include "libxr_def.hpp"
#include "sg200x_ll_adc.h"

namespace LibXR
{

/**
 * @brief SG200x C906L SARADC driver.
 *
 * The SG200x exposes three top-domain channels and three RTC-domain channels.
 * A conversion is a single-shot, polled transaction; the hardware does not
 * provide a DMA sample stream for this peripheral.
 */
class SG200XADC final
{
 public:
  enum class ChannelId : uint8_t
  {
    ADC1 = 1u,
    ADC2,
    ADC3,
    PWR_ADC1,
    PWR_ADC2,
    PWR_ADC3,
  };

  enum class Reference : uint8_t
  {
    INTERNAL = 0u,
    EXTERNAL_VDD18A = 1u,
  };

  struct Config
  {
    float reference_voltage = 1.8f;
    Reference reference = Reference::INTERNAL;
    uint8_t clock_divider = SARADC_CYCLE_DIVIDER_MAX;
    uint32_t timeout_iterations = 100000u;
  };

  class Channel final : public ADC
  {
   public:
    float Read() override;

    [[nodiscard]] uint8_t ChannelNumber() const noexcept { return channel_number_; }

   private:
    friend class SG200XADC;
    Channel() = default;
    Channel(SG200XADC* parent, uint8_t index, uint8_t channel_number)
        : parent_(parent), index_(index), channel_number_(channel_number)
    {
    }

    SG200XADC* parent_ = nullptr;
    uint8_t index_ = 0u;
    uint8_t channel_number_ = 0u;
  };

  static constexpr uintptr_t ACTIVE_BASE = SARADC_BASE;
  static constexpr uintptr_t RTC_BASE = RTC_SARADC_BASE;
  static constexpr uintptr_t RTC_CTRL_BASE = ::RTC_CTRL_BASE;
  static constexpr uint32_t CHANNEL_COUNT = SARADC_COUNT * SARADC_CHANNEL_COUNT;
  static constexpr uint16_t MAX_RAW = SARADC_RESULT_DATA_MASK;

  /**
   * @param channels Logical channels in the 1..6 numbering from the TRM.
   * @param config Conversion settings. A divider of 15 gives a conservative
   *               1.5625 MHz SARADC clock from the default 25 MHz XTAL.
   */
  explicit SG200XADC(std::initializer_list<uint8_t> channels = {1u, 2u, 3u, 4u, 5u, 6u},
                     Config config = {1.8f, Reference::INTERNAL, 15u, 100000u});

  Channel& GetChannel(uint8_t index) noexcept;
  float ReadChannel(uint8_t index) noexcept;
  ErrorCode ReadRaw(uint8_t index, uint16_t& value) noexcept;

  [[nodiscard]] bool IsValid() const noexcept { return valid_; }
  [[nodiscard]] uint8_t ChannelCount() const noexcept { return channel_count_; }

 private:
  static SARADC_Type* DomainInstance(uint8_t channel) noexcept;
  static uint8_t DomainChannel(uint8_t channel) noexcept;
  static ErrorCode EnableClocksAndReset() noexcept;
  ErrorCode ConfigureDomain(SARADC_Type* adc) const noexcept;
  ErrorCode WaitIdle(const SARADC_Type* adc) const noexcept;

  Channel channels_[CHANNEL_COUNT]{};
  uint8_t channel_numbers_[CHANNEL_COUNT]{};
  uint8_t channel_count_ = 0u;
  float reference_voltage_ = 0.0f;
  Reference reference_ = Reference::INTERNAL;
  uint8_t clock_divider_ = 0u;
  uint32_t timeout_iterations_ = 0u;
  std::atomic<bool> conversion_active_{false};
  bool valid_ = false;
};

}  // namespace LibXR

#pragma once

#include <atomic>
#include <cstdint>
#include <initializer_list>

#include "adc.hpp"
#include "libxr_def.hpp"

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
    uint8_t clock_divider = 15u;
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

  static constexpr uintptr_t ACTIVE_BASE = 0x030F0000u;
  static constexpr uintptr_t RTC_BASE = 0x0502C000u;
  static constexpr uintptr_t RTC_CTRL_BASE = 0x05025000u;
  static constexpr uint32_t CHANNEL_COUNT = 6u;
  static constexpr uint16_t MAX_RAW = 4095u;

  /**
   * @param channels Logical channels in the 1..6 numbering from the TRM.
   * @param config Conversion settings. A divider of 15 gives a conservative
   *               1.5625 MHz SARADC clock from the default 25 MHz XTAL.
   */
  explicit SG200XADC(
      std::initializer_list<uint8_t> channels = {1u, 2u, 3u, 4u, 5u, 6u},
      Config config = {1.8f, Reference::INTERNAL, 15u, 100000u});

  Channel& GetChannel(uint8_t index) noexcept;
  float ReadChannel(uint8_t index) noexcept;
  ErrorCode ReadRaw(uint8_t index, uint16_t& value) noexcept;

  [[nodiscard]] bool IsValid() const noexcept { return valid_; }
  [[nodiscard]] uint8_t ChannelCount() const noexcept { return channel_count_; }

 private:
  static constexpr uint32_t REG_CTRL = 0x04u;
  static constexpr uint32_t REG_STATUS = 0x08u;
  static constexpr uint32_t REG_CYC_SET = 0x0Cu;
  static constexpr uint32_t REG_RESULT_BASE = 0x14u;
  static constexpr uint32_t REG_INTR_EN = 0x20u;
  static constexpr uint32_t REG_INTR_CLR = 0x24u;
  static constexpr uint32_t REG_TEST = 0x30u;
  static constexpr uint32_t RTC_REG_RESET = 0x18u;
  static constexpr uint32_t RTC_REG_CLOCK_MUX = 0x1Cu;
  static constexpr uint32_t STATUS_BUSY = 1u << 0u;
  static constexpr uint32_t RESULT_VALID = 1u << 15u;
  static constexpr uint32_t CHANNEL_SELECT_MASK = 0xFu << 4u;
  static constexpr uint32_t TRIGGER = 1u << 0u;
  static constexpr uint32_t RTC_SARADC_RESETN = 1u << 17u;
  static constexpr uint32_t RTC_SARADC_CLOCK_MUX = 1u << 20u;
  static constexpr uint32_t TEST_REFERENCE_MASK = 1u << 2u;
  static constexpr uint32_t TEST_REFERENCE_SHIFT = 2u;
  static constexpr uint32_t CYCLE_SETTLE_MASK = 0x1Fu;
  static constexpr uint32_t CYCLE_SAMPLE_MASK = 0xFu << 8u;
  static constexpr uint32_t CYCLE_DIVIDER_MASK = 0xFu << 12u;
  static constexpr uint32_t CYCLE_COMPARE_MASK = 0xFu << 16u;
  static constexpr uint32_t CYCLE_SETTLE_DEFAULT = 0x0Fu;
  static constexpr uint32_t CYCLE_SAMPLE_DEFAULT = 0x03u << 8u;
  static constexpr uint32_t CYCLE_COMPARE_DEFAULT = 0x0Bu << 16u;
  static uintptr_t DomainBase(uint8_t channel) noexcept;
  static uint8_t DomainChannel(uint8_t channel) noexcept;
  static void EnableClocksAndReset() noexcept;
  void ConfigureDomain(uintptr_t base) const noexcept;
  ErrorCode WaitIdle(uintptr_t base) const noexcept;

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

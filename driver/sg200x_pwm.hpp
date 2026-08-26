#pragma once

#include <cstdint>

#include "pwm.hpp"

namespace LibXR
{

/**
 * @brief SG200x PWM controller driver.
 *
 * The TRM exposes four PWM controllers at 0x03060000..0x03063000. Each
 * controller owns four outputs, so Channel::PWM0..PWM15 are a global channel
 * numbering across those controllers. The output pin must be muxed to the
 * selected PWM function by board startup code, or with the optional pinmux
 * descriptor passed to the constructor.
 */
class SG200XPWM final : public PWM
{
 public:
  static constexpr uint32_t INVALID_PINMUX = UINT32_MAX;

  enum class Channel : uint8_t
  {
    PWM0 = 0u,
    PWM1,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    PWM15,
  };

  struct PinmuxConfiguration
  {
    uint32_t offset = INVALID_PINMUX;
    uint8_t function = 0u;
  };

  static constexpr uintptr_t PWM0_BASE = 0x03060000u;
  static constexpr uintptr_t PWM1_BASE = 0x03061000u;
  static constexpr uintptr_t PWM2_BASE = 0x03062000u;
  static constexpr uintptr_t PWM3_BASE = 0x03063000u;
  static constexpr uintptr_t PINMUX_BASE = 0x03001000u;
  static constexpr uint32_t PWM_CLOCK_HZ = 100000000u;
  static constexpr uint32_t MAX_PERIOD_TICKS = 0x3FFFFFFFu;
  static constexpr uint8_t CHANNEL_COUNT = 16u;

  /** Dedicated SDK mapping for the PWM0_BUCK package pad. */
  static constexpr PinmuxConfiguration PWM0_BUCK_PINMUX{0xECu, 0u};

  explicit SG200XPWM(Channel channel, uint32_t clock_hz = PWM_CLOCK_HZ);
  SG200XPWM(Channel channel, uint32_t clock_hz, PinmuxConfiguration pinmux);
  explicit SG200XPWM(uint8_t channel, uint32_t clock_hz = PWM_CLOCK_HZ);
  SG200XPWM(uint8_t channel, uint32_t clock_hz, PinmuxConfiguration pinmux);

  ErrorCode SetDutyCycle(float value) override;
  ErrorCode SetConfig(Configuration config) override;
  ErrorCode Enable() override;
  ErrorCode Disable() override;

  /** @brief Set output polarity while the channel is stopped. */
  ErrorCode SetPolarity(bool active_high);

  [[nodiscard]] bool IsValid() const noexcept { return pwm_base_ != 0u; }
  [[nodiscard]] uint32_t ClockHz() const noexcept { return clock_hz_; }
  [[nodiscard]] uint32_t PeriodTicks() const noexcept { return period_ticks_; }
  [[nodiscard]] float DutyCycle() const noexcept { return duty_cycle_; }

 private:
  static constexpr uint32_t REG_HLPERIOD = 0x00u;
  static constexpr uint32_t REG_PERIOD = 0x04u;
  static constexpr uint32_t REG_POLARITY = 0x40u;
  static constexpr uint32_t REG_PWMSTART = 0x44u;
  static constexpr uint32_t REG_PWMUPDATE = 0x4Cu;
  static constexpr uint32_t REG_PWM_OE = 0xD0u;
  static constexpr uint32_t CHANNEL_STRIDE = 0x08u;

  static constexpr uint32_t PINMUX_FUNCTION_MASK = 0x7u;
  static constexpr uintptr_t CONTROLLER_STRIDE = 0x1000u;
  static constexpr uint8_t CHANNELS_PER_CONTROLLER = 4u;

  static uintptr_t ControllerBase(uint8_t channel) noexcept;
  static uint8_t LocalChannel(uint8_t channel) noexcept;
  static void ConfigurePinmux(const PinmuxConfiguration& pinmux) noexcept;
  static uint32_t DutyToHighTicks(float duty, uint32_t period) noexcept;

  void WritePeriodRegisters() noexcept;
  void ApplyDynamicUpdate() noexcept;

  uintptr_t pwm_base_ = 0u;
  uint8_t channel_ = 0u;
  uint32_t channel_mask_ = 0u;
  uint32_t clock_hz_ = 0u;
  uint32_t period_ticks_ = 0u;
  uint32_t high_ticks_ = 1u;
  float duty_cycle_ = 0.0f;
  PinmuxConfiguration pinmux_{};
  bool active_high_ = true;
  bool enabled_ = false;
};

}  // namespace LibXR

#pragma once

#include <cstdint>

#include "pwm.hpp"
#include "sg200x_ll_pwm.h"

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

  static constexpr uint32_t PWM_CLOCK_HZ = 100000000u;
  static constexpr uint32_t MAX_PERIOD_TICKS = PWM_PERIOD_MAX;
  static constexpr uint8_t CHANNEL_COUNT = PWM_CHANNEL_COUNT;

  /** Dedicated SDK mapping for the PWM0_BUCK package pad. */
  static constexpr PinmuxConfiguration PWM0_BUCK_PINMUX{PINMUX_PWM0_BUCK_OFFSET,
                                                        PINMUX_PWM0_BUCK_PWM0_FUNCTION};

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

  [[nodiscard]] bool IsValid() const noexcept { return regs_ != nullptr; }
  [[nodiscard]] uint32_t ClockHz() const noexcept { return clock_hz_; }
  [[nodiscard]] uint32_t PeriodTicks() const noexcept { return period_ticks_; }
  [[nodiscard]] float DutyCycle() const noexcept { return duty_cycle_; }

 private:
  static PWM_Type* ControllerInstance(uint8_t channel) noexcept;
  static uint8_t LocalChannel(uint8_t channel) noexcept;
  static void ConfigurePinmux(const PinmuxConfiguration& pinmux) noexcept;
  static uint32_t DutyToHighTicks(float duty, uint32_t period) noexcept;

  void WritePeriodRegisters() noexcept;
  void ApplyDynamicUpdate() noexcept;

  PWM_Type* regs_ = nullptr;
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

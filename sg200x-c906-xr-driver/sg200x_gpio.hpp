#pragma once

#include <cstdint>

#include "gpio.hpp"

namespace LibXR
{

/**
 * @brief SG200x C906 DesignWare GPIO driver.
 *
 * GPIO0 through GPIO3 each expose one 32-bit port. The driver applies the
 * SoC-defined GPIO function for pins with a known pad mapping; other board
 * pinmux setup remains the responsibility of startup firmware.
 */
class SG200XGPIO final : public GPIO
{
 public:
  enum class Bank : uint8_t
  {
    A,
    B,
    C,
    D,
  };

  static constexpr uintptr_t GPIO0_BASE = 0x03020000u;
  static constexpr uintptr_t GPIO1_BASE = 0x03021000u;
  static constexpr uintptr_t GPIO2_BASE = 0x03022000u;
  static constexpr uintptr_t GPIO3_BASE = 0x03023000u;
  static constexpr uintptr_t PINMUX_BASE = 0x03001000u;
  static constexpr uint32_t GPIO0_IRQ = 41u;
  static constexpr uint32_t GPIO1_IRQ = 42u;
  static constexpr uint32_t GPIO2_IRQ = 43u;
  static constexpr uint32_t GPIO3_IRQ = 44u;
  static constexpr uint32_t INVALID_PINMUX = UINT32_MAX;
  explicit SG200XGPIO(Bank bank, uint8_t pin);

  bool Read() override;
  void Write(bool value) override;
  ErrorCode EnableInterrupt() override;
  ErrorCode DisableInterrupt() override;
  ErrorCode SetConfig(Configuration config) override;

  /** @brief Dispatch a GPIO controller interrupt from a board IRQ handler. */
  static void CheckInterrupt(uintptr_t gpio_base);

 private:
  static constexpr uint32_t REG_DR = 0x00u;
  static constexpr uint32_t REG_DDR = 0x04u;
  static constexpr uint32_t REG_INTEN = 0x30u;
  static constexpr uint32_t REG_INTMASK = 0x34u;
  static constexpr uint32_t REG_INTTYPE_LEVEL = 0x38u;
  static constexpr uint32_t REG_INT_POLARITY = 0x3Cu;
  static constexpr uint32_t REG_INTSTATUS = 0x40u;
  static constexpr uint32_t REG_EOI = 0x4Cu;
  static constexpr uint32_t REG_EXT_PORTA = 0x50u;

  static constexpr uint8_t CONTROLLER_COUNT = 4u;
  static constexpr uint8_t PIN_COUNT = 32u;

  static uint8_t ControllerIndex(uintptr_t gpio_base) noexcept;
  static uintptr_t BankBase(Bank bank) noexcept;
  static uint32_t PinmuxOffset(Bank bank, uint8_t pin) noexcept;
  static uint32_t DefaultIrq(uint8_t controller) noexcept;
  static int InterruptHandler(int irq, void* argument);

  uintptr_t gpio_base_ = 0u;
  uint8_t pin_ = 0u;
  uint32_t pin_mask_ = 0u;
  uint32_t pinmux_offset_ = INVALID_PINMUX;
  uint32_t irq_ = 0u;
  Direction direction_ = Direction::INPUT;
  bool interrupt_enabled_ = false;

  static SG200XGPIO* instances_[CONTROLLER_COUNT][PIN_COUNT];
  static bool irq_registered_[CONTROLLER_COUNT];
};

}  // namespace LibXR

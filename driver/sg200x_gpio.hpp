#pragma once

#include <atomic>
#include <cstdint>

#include "gpio.hpp"
#include "sg200x_ll_gpio.h"

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
  static_assert(std::atomic<SG200XGPIO*>::is_always_lock_free,
                "SG200XGPIO requires lock-free pointer atomics");
  static_assert(std::atomic<bool>::is_always_lock_free,
                "SG200XGPIO requires lock-free boolean atomics");

  enum class Bank : uint8_t
  {
    A,
    B,
    C,
    D,
  };

  static constexpr uint32_t GPIO0_IRQ = IRQ_GPIO0;
  static constexpr uint32_t GPIO1_IRQ = IRQ_GPIO1;
  static constexpr uint32_t GPIO2_IRQ = IRQ_GPIO2;
  static constexpr uint32_t GPIO3_IRQ = IRQ_GPIO3;
  static constexpr uint32_t INVALID_PINMUX = UINT32_MAX;
  explicit SG200XGPIO(Bank bank, uint8_t pin);
  ~SG200XGPIO() override;

  SG200XGPIO(const SG200XGPIO&) = delete;
  SG200XGPIO& operator=(const SG200XGPIO&) = delete;
  SG200XGPIO(SG200XGPIO&&) = delete;
  SG200XGPIO& operator=(SG200XGPIO&&) = delete;

  bool Read() override;
  void Write(bool value) override;
  ErrorCode EnableInterrupt() override;
  ErrorCode DisableInterrupt() override;
  ErrorCode SetConfig(Configuration config) override;

  /** @brief Dispatch a GPIO controller interrupt from a board IRQ handler. */
  static void CheckInterrupt(uintptr_t gpio_base);

 private:
  enum class IrqRegistrationState : uint8_t
  {
    UNREGISTERED,
    INITIALIZING,
    READY,
  };
  static_assert(std::atomic<IrqRegistrationState>::is_always_lock_free,
                "SG200XGPIO requires lock-free IRQ state atomics");

  static constexpr uint8_t CONTROLLER_COUNT = GPIO_COUNT;
  static constexpr uint8_t PIN_COUNT = GPIO_PIN_COUNT;

  static uint8_t ControllerIndex(const GPIO_Type* gpio) noexcept;
  static GPIO_Type* BankInstance(Bank bank) noexcept;
  static uint32_t PinmuxOffset(Bank bank, uint8_t pin) noexcept;
  static uint32_t DefaultIrq(uint8_t controller) noexcept;
  static int InterruptHandler(int irq, void* argument);

  GPIO_Type* regs_ = nullptr;
  uint8_t pin_ = 0u;
  uint32_t pin_mask_ = 0u;
  uint32_t pinmux_offset_ = INVALID_PINMUX;
  uint32_t irq_ = 0u;
  Direction direction_ = Direction::INPUT;
  std::atomic<bool> interrupt_enabled_{false};

  static std::atomic<SG200XGPIO*> instances_[CONTROLLER_COUNT][PIN_COUNT];
  static std::atomic<IrqRegistrationState> irq_registration_[CONTROLLER_COUNT];
};

}  // namespace LibXR

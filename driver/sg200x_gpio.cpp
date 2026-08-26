#include "sg200x_gpio.hpp"

#include "sg200x_mmio.hpp"

// The Vendor SDK exports this Linux-style IRQ registration symbol from its
// C906 interrupt support. Keep the declaration at global scope so the weak
// reference has external linkage and can resolve to that implementation.
extern "C" int request_irq(unsigned int irqn, int (*handler)(int, void*),
                           unsigned long flags, const char* name, void* argument)
    __attribute__((weak));

namespace LibXR
{
namespace
{
void ConfigurePinmux(uint32_t offset)
{
  if (offset == SG200XGPIO::INVALID_PINMUX)
  {
    return;
  }

  auto& reg = Register32(SG200XGPIO::PINMUX_BASE, offset);
  reg = (reg & ~0x7u) | 3u;
}

}  // namespace

SG200XGPIO* SG200XGPIO::instances_[CONTROLLER_COUNT][PIN_COUNT] = {};
bool SG200XGPIO::irq_registered_[CONTROLLER_COUNT] = {};

uint8_t SG200XGPIO::ControllerIndex(uintptr_t gpio_base) noexcept
{
  switch (gpio_base)
  {
    case GPIO0_BASE:
      return 0u;
    case GPIO1_BASE:
      return 1u;
    case GPIO2_BASE:
      return 2u;
    case GPIO3_BASE:
      return 3u;
    default:
      return CONTROLLER_COUNT;
  }
}

uintptr_t SG200XGPIO::BankBase(Bank bank) noexcept
{
  switch (bank)
  {
    case Bank::A:
      return GPIO0_BASE;
    case Bank::B:
      return GPIO1_BASE;
    case Bank::C:
      return GPIO2_BASE;
    case Bank::D:
      return GPIO3_BASE;
  }
  return 0u;
}

uint32_t SG200XGPIO::PinmuxOffset(Bank bank, uint8_t pin) noexcept
{
  // SD0_PWR_EN is the SoC pad carrying GPIOA_14 on SG200x.
  if (bank == Bank::A && pin == 14u)
  {
    return 0x38u;
  }
  // LicheeRV Nano labels these EMMC pads as software SPI4.
  if (bank == Bank::A)
  {
    switch (pin)
    {
      case 22u:  // EMMC_CLK / SPI4_SCK
        return 0x50u;
      case 23u:  // EMMC_CMD / SPI4_MISO
        return 0x5Cu;
      case 24u:  // EMMC_DAT1 / SPI4_CS
        return 0x60u;
      case 25u:  // EMMC_DAT0 / SPI4_MOSI
        return 0x54u;
      default:
        break;
    }
  }
  return INVALID_PINMUX;
}

uint32_t SG200XGPIO::DefaultIrq(uint8_t controller) noexcept
{
  return GPIO0_IRQ + controller;
}

SG200XGPIO::SG200XGPIO(Bank bank, uint8_t pin) : gpio_base_(BankBase(bank)), pin_(pin)
{
  const uint8_t controller = ControllerIndex(gpio_base_);
  if (controller >= CONTROLLER_COUNT || pin_ >= PIN_COUNT)
  {
    gpio_base_ = 0u;
    return;
  }

  pin_mask_ = static_cast<uint32_t>(1u) << pin_;
  pinmux_offset_ = PinmuxOffset(bank, pin_);
  irq_ = DefaultIrq(controller);
  instances_[controller][pin_] = this;
}

bool SG200XGPIO::Read()
{
  return gpio_base_ != 0u && (Register32(gpio_base_, REG_EXT_PORTA) & pin_mask_) != 0u;
}

void SG200XGPIO::Write(bool value)
{
  if (gpio_base_ == 0u)
  {
    return;
  }

  ConfigurePinmux(pinmux_offset_);

  auto& direction = Register32(gpio_base_, REG_DDR);
  if (direction_ == Direction::OUTPUT_OPEN_DRAIN && value)
  {
    // An open-drain high level is released by switching the pin to input.
    direction &= ~pin_mask_;
  }
  else if (direction_ == Direction::OUTPUT_OPEN_DRAIN ||
           direction_ == Direction::OUTPUT_PUSH_PULL)
  {
    direction |= pin_mask_;
  }

  // Configure output-enable before changing the data latch.  This is the
  // sequence used by Sophgo's C906 GPIO helper and avoids a transient/ignored
  // write on the DesignWare GPIO block.
  auto& data = Register32(gpio_base_, REG_DR);
  // The TRM specifies that DR reads back the output latch, so preserve the
  // other GPIO bits when changing this pin.
  data = (data & ~pin_mask_) | (value ? pin_mask_ : 0u);
}

ErrorCode SG200XGPIO::SetConfig(Configuration config)
{
  if (gpio_base_ == 0u)
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.pull != Pull::NONE)
  {
    // The public C906 register description exposes no pad bias register.
    return ErrorCode::NOT_SUPPORT;
  }
  if (config.direction == Direction::FALL_RISING_INTERRUPT)
  {
    // The GPIO block has one polarity bit per pin and cannot select both edges.
    return ErrorCode::NOT_SUPPORT;
  }

  const uint8_t controller = ControllerIndex(gpio_base_);
  if (controller >= CONTROLLER_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }

  ConfigurePinmux(pinmux_offset_);

  direction_ = config.direction;
  auto& interrupt_enable = Register32(gpio_base_, REG_INTEN);
  auto& interrupt_mask = Register32(gpio_base_, REG_INTMASK);
  auto& interrupt_type = Register32(gpio_base_, REG_INTTYPE_LEVEL);
  auto& interrupt_polarity = Register32(gpio_base_, REG_INT_POLARITY);
  auto& direction = Register32(gpio_base_, REG_DDR);

  interrupt_enabled_ = false;
  interrupt_enable &= ~pin_mask_;
  interrupt_mask |= pin_mask_;
  Register32(gpio_base_, REG_EOI) = pin_mask_;

  switch (config.direction)
  {
    case Direction::INPUT:
      direction &= ~pin_mask_;
      interrupt_type &= ~pin_mask_;
      break;
    case Direction::OUTPUT_PUSH_PULL:
      direction |= pin_mask_;
      interrupt_type &= ~pin_mask_;
      break;
    case Direction::OUTPUT_OPEN_DRAIN:
      direction |= pin_mask_;
      interrupt_type &= ~pin_mask_;
      Register32(gpio_base_, REG_DR) &= ~pin_mask_;
      break;
    case Direction::RISING_INTERRUPT:
      direction &= ~pin_mask_;
      interrupt_type |= pin_mask_;
      interrupt_polarity |= pin_mask_;
      break;
    case Direction::FALL_INTERRUPT:
      direction &= ~pin_mask_;
      interrupt_type |= pin_mask_;
      interrupt_polarity &= ~pin_mask_;
      break;
    case Direction::FALL_RISING_INTERRUPT:
      return ErrorCode::NOT_SUPPORT;
  }

  return ErrorCode::OK;
}

ErrorCode SG200XGPIO::EnableInterrupt()
{
  if (gpio_base_ == 0u)
  {
    return ErrorCode::ARG_ERR;
  }
  if (direction_ != Direction::RISING_INTERRUPT &&
      direction_ != Direction::FALL_INTERRUPT)
  {
    return ErrorCode::STATE_ERR;
  }

  const uint8_t controller = ControllerIndex(gpio_base_);
  if (controller >= CONTROLLER_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }
  if (!irq_registered_[controller])
  {
    if (request_irq == nullptr ||
        request_irq(irq_, &SG200XGPIO::InterruptHandler, 0u, "sg200x-gpio",
                    reinterpret_cast<void*>(gpio_base_)) != 0)
    {
      return ErrorCode::NOT_SUPPORT;
    }
    irq_registered_[controller] = true;
  }

  Register32(gpio_base_, REG_EOI) = pin_mask_;
  Register32(gpio_base_, REG_INTMASK) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTEN) |= pin_mask_;
  interrupt_enabled_ = true;
  return ErrorCode::OK;
}

ErrorCode SG200XGPIO::DisableInterrupt()
{
  if (gpio_base_ == 0u)
  {
    return ErrorCode::ARG_ERR;
  }

  Register32(gpio_base_, REG_INTEN) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTMASK) |= pin_mask_;
  Register32(gpio_base_, REG_EOI) = pin_mask_;
  interrupt_enabled_ = false;
  return ErrorCode::OK;
}

void SG200XGPIO::CheckInterrupt(uintptr_t gpio_base)
{
  const uint8_t controller = ControllerIndex(gpio_base);
  if (controller >= CONTROLLER_COUNT)
  {
    return;
  }

  const uint32_t pending = Register32(gpio_base, REG_INTSTATUS);
  if (pending != 0u)
  {
    Register32(gpio_base, REG_EOI) = pending;
  }

  for (uint8_t pin = 0u; pin < PIN_COUNT; ++pin)
  {
    if ((pending & (static_cast<uint32_t>(1u) << pin)) != 0u &&
        instances_[controller][pin] != nullptr &&
        instances_[controller][pin]->interrupt_enabled_)
    {
      instances_[controller][pin]->callback_.Run(true);
    }
  }
}

int SG200XGPIO::InterruptHandler(int, void* argument)
{
  CheckInterrupt(reinterpret_cast<uintptr_t>(argument));
  return 0;
}

}  // namespace LibXR

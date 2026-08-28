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
constexpr uintptr_t GPIO0_BASE = 0x03020000u;
constexpr uintptr_t GPIO1_BASE = 0x03021000u;
constexpr uintptr_t GPIO2_BASE = 0x03022000u;
constexpr uintptr_t GPIO3_BASE = 0x03023000u;
constexpr uintptr_t GPIO_CONTROLLER_STRIDE = 0x1000u;
constexpr uintptr_t PINMUX_BASE = 0x03001000u;

void IoFence() noexcept { asm volatile("fence iorw, iorw" ::: "memory"); }

void ConfigurePinmux(uint32_t offset)
{
  if (offset == SG200XGPIO::INVALID_PINMUX)
  {
    return;
  }

  auto& reg = Register32(PINMUX_BASE, offset);
  reg = (reg & ~0x7u) | 3u;
}

}  // namespace

std::atomic<SG200XGPIO*>
    SG200XGPIO::instances_[CONTROLLER_COUNT][PIN_COUNT]{};
std::atomic<SG200XGPIO::IrqRegistrationState>
    SG200XGPIO::irq_registration_[CONTROLLER_COUNT]{};

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

SG200XGPIO::SG200XGPIO(Bank bank, uint8_t pin)
    : gpio_base_(BankBase(bank)), pin_(pin)
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
  SG200XGPIO* expected = nullptr;
  if (!instances_[controller][pin_].compare_exchange_strong(
          expected, this, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    gpio_base_ = 0u;
    pin_mask_ = 0u;
    return;
  }
}

SG200XGPIO::~SG200XGPIO()
{
  const uint8_t controller = ControllerIndex(gpio_base_);
  if (controller >= CONTROLLER_COUNT || pin_ >= PIN_COUNT)
  {
    return;
  }

  interrupt_enabled_.store(false, std::memory_order_release);
  IoFence();
  Register32(gpio_base_, REG_INTEN) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTMASK) |= pin_mask_;
  Register32(gpio_base_, REG_EOI) = pin_mask_;
  IoFence();
  SG200XGPIO* expected = this;
  (void)instances_[controller][pin_].compare_exchange_strong(
      expected, nullptr, std::memory_order_acq_rel, std::memory_order_acquire);
  gpio_base_ = 0u;
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

  auto& data = Register32(gpio_base_, REG_DR);
  if (direction_ == Direction::OUTPUT_OPEN_DRAIN && value)
  {
    // An open-drain high level is released by switching the pin to input.
    Register32(gpio_base_, REG_DDR) &= ~pin_mask_;
    data |= pin_mask_;
    return;
  }
  if (direction_ == Direction::OUTPUT_OPEN_DRAIN)
  {
    // Never enable the push-pull output while its latch still contains high.
    data &= ~pin_mask_;
    IoFence();
    Register32(gpio_base_, REG_DDR) |= pin_mask_;
    return;
  }
  if (direction_ == Direction::OUTPUT_PUSH_PULL)
  {
    Register32(gpio_base_, REG_DDR) |= pin_mask_;
  }

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
  interrupt_enabled_.store(false, std::memory_order_release);
  IoFence();
  Register32(gpio_base_, REG_INTEN) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTMASK) |= pin_mask_;
  Register32(gpio_base_, REG_EOI) = pin_mask_;

  switch (config.direction)
  {
    case Direction::INPUT:
      Register32(gpio_base_, REG_DDR) &= ~pin_mask_;
      Register32(gpio_base_, REG_INTTYPE_LEVEL) &= ~pin_mask_;
      break;
    case Direction::OUTPUT_PUSH_PULL:
      Register32(gpio_base_, REG_DDR) |= pin_mask_;
      Register32(gpio_base_, REG_INTTYPE_LEVEL) &= ~pin_mask_;
      break;
    case Direction::OUTPUT_OPEN_DRAIN:
      Register32(gpio_base_, REG_DR) &= ~pin_mask_;
      IoFence();
      Register32(gpio_base_, REG_DDR) |= pin_mask_;
      Register32(gpio_base_, REG_INTTYPE_LEVEL) &= ~pin_mask_;
      break;
    case Direction::RISING_INTERRUPT:
      Register32(gpio_base_, REG_DDR) &= ~pin_mask_;
      Register32(gpio_base_, REG_INTTYPE_LEVEL) |= pin_mask_;
      Register32(gpio_base_, REG_INT_POLARITY) |= pin_mask_;
      break;
    case Direction::FALL_INTERRUPT:
      Register32(gpio_base_, REG_DDR) &= ~pin_mask_;
      Register32(gpio_base_, REG_INTTYPE_LEVEL) |= pin_mask_;
      Register32(gpio_base_, REG_INT_POLARITY) &= ~pin_mask_;
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
  IrqRegistrationState registration =
      irq_registration_[controller].load(std::memory_order_acquire);
  if (registration == IrqRegistrationState::UNREGISTERED)
  {
    IrqRegistrationState expected = IrqRegistrationState::UNREGISTERED;
    if (!irq_registration_[controller].compare_exchange_strong(
            expected, IrqRegistrationState::INITIALIZING,
            std::memory_order_acq_rel, std::memory_order_acquire))
    {
      return ErrorCode::BUSY;
    }
    if (request_irq == nullptr ||
        request_irq(irq_, &SG200XGPIO::InterruptHandler, 0u, "sg200x-gpio",
                    nullptr) != 0)
    {
      irq_registration_[controller].store(IrqRegistrationState::UNREGISTERED,
                                           std::memory_order_release);
      return ErrorCode::NOT_SUPPORT;
    }
    irq_registration_[controller].store(IrqRegistrationState::READY,
                                         std::memory_order_release);
  }
  else if (registration != IrqRegistrationState::READY)
  {
    return ErrorCode::BUSY;
  }

  interrupt_enabled_.store(true, std::memory_order_release);
  IoFence();
  Register32(gpio_base_, REG_EOI) = pin_mask_;
  Register32(gpio_base_, REG_INTMASK) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTEN) |= pin_mask_;
  IoFence();
  return ErrorCode::OK;
}

ErrorCode SG200XGPIO::DisableInterrupt()
{
  if (gpio_base_ == 0u)
  {
    return ErrorCode::ARG_ERR;
  }

  interrupt_enabled_.store(false, std::memory_order_release);
  IoFence();
  Register32(gpio_base_, REG_INTEN) &= ~pin_mask_;
  Register32(gpio_base_, REG_INTMASK) |= pin_mask_;
  Register32(gpio_base_, REG_EOI) = pin_mask_;
  IoFence();
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
    SG200XGPIO* const instance =
        instances_[controller][pin].load(std::memory_order_acquire);
    if ((pending & (static_cast<uint32_t>(1u) << pin)) != 0u &&
        instance != nullptr &&
        instance->interrupt_enabled_.load(std::memory_order_acquire))
    {
      instance->callback_.Run(true);
    }
  }
}

int SG200XGPIO::InterruptHandler(int irq, void*)
{
  if (irq < static_cast<int>(GPIO0_IRQ) ||
      irq >= static_cast<int>(GPIO0_IRQ + CONTROLLER_COUNT))
  {
    return 0;
  }
  const auto controller = static_cast<uint8_t>(irq - static_cast<int>(GPIO0_IRQ));
  CheckInterrupt(GPIO0_BASE + static_cast<uintptr_t>(controller) *
                                   GPIO_CONTROLLER_STRIDE);
  return 0;
}

}  // namespace LibXR

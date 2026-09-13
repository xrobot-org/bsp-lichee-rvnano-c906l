#include "sg200x_gpio.hpp"

#include "sg200x_ll_csr.h"
#include "sg200x_ll_pinmux.h"

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
  switch (offset)
  {
    case PINMUX_SD0_PWR_EN_OFFSET:
      (void)sg200x_ll_pinmux_function_set(offset, PINMUX_SD0_PWR_EN_GPIOA14_FUNCTION);
      break;
    case PINMUX_EMMC_CLK_OFFSET:
      (void)sg200x_ll_pinmux_function_set(offset, PINMUX_EMMC_CLK_GPIOA22_FUNCTION);
      break;
    case PINMUX_EMMC_CMD_OFFSET:
      (void)sg200x_ll_pinmux_function_set(offset, PINMUX_EMMC_CMD_GPIOA23_FUNCTION);
      break;
    case PINMUX_EMMC_DAT1_OFFSET:
      (void)sg200x_ll_pinmux_function_set(offset, PINMUX_EMMC_DAT1_GPIOA24_FUNCTION);
      break;
    case PINMUX_EMMC_DAT0_OFFSET:
      (void)sg200x_ll_pinmux_function_set(offset, PINMUX_EMMC_DAT0_GPIOA25_FUNCTION);
      break;
    default:
      break;
  }
}

}  // namespace

std::atomic<SG200XGPIO*> SG200XGPIO::instances_[CONTROLLER_COUNT][PIN_COUNT]{};
std::atomic<SG200XGPIO::IrqRegistrationState>
    SG200XGPIO::irq_registration_[CONTROLLER_COUNT]{};

uint8_t SG200XGPIO::ControllerIndex(const GPIO_Type* gpio) noexcept
{
  return static_cast<uint8_t>(sg200x_ll_gpio_index_get(gpio));
}

GPIO_Type* SG200XGPIO::BankInstance(Bank bank) noexcept
{
  return sg200x_ll_gpio_get(static_cast<uint32_t>(bank));
}

uint32_t SG200XGPIO::PinmuxOffset(Bank bank, uint8_t pin) noexcept
{
  // SD0_PWR_EN is the SoC pad carrying GPIOA_14 on SG200x.
  if (bank == Bank::A && pin == 14u)
  {
    return PINMUX_SD0_PWR_EN_OFFSET;
  }
  // LicheeRV Nano labels these EMMC pads as software SPI4.
  if (bank == Bank::A)
  {
    switch (pin)
    {
      case 22u:  // EMMC_CLK / SPI4_SCK
        return PINMUX_EMMC_CLK_OFFSET;
      case 23u:  // EMMC_CMD / SPI4_MISO
        return PINMUX_EMMC_CMD_OFFSET;
      case 24u:  // EMMC_DAT1 / SPI4_CS
        return PINMUX_EMMC_DAT1_OFFSET;
      case 25u:  // EMMC_DAT0 / SPI4_MOSI
        return PINMUX_EMMC_DAT0_OFFSET;
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

SG200XGPIO::SG200XGPIO(Bank bank, uint8_t pin) : regs_(BankInstance(bank)), pin_(pin)
{
  const uint8_t controller = ControllerIndex(regs_);
  if (controller >= CONTROLLER_COUNT || pin_ >= PIN_COUNT)
  {
    regs_ = nullptr;
    return;
  }

  pin_mask_ = static_cast<uint32_t>(1u) << pin_;
  pinmux_offset_ = PinmuxOffset(bank, pin_);
  irq_ = DefaultIrq(controller);
  SG200XGPIO* expected = nullptr;
  if (!instances_[controller][pin_].compare_exchange_strong(
          expected, this, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    regs_ = nullptr;
    pin_mask_ = 0u;
    return;
  }
}

SG200XGPIO::~SG200XGPIO()
{
  const uint8_t controller = ControllerIndex(regs_);
  if (controller >= CONTROLLER_COUNT || pin_ >= PIN_COUNT)
  {
    return;
  }

  interrupt_enabled_.store(false, std::memory_order_release);
  sg200x_ll_csr_fence_io();
  sg200x_ll_gpio_interrupt_enable(regs_, pin_mask_, false);
  sg200x_ll_gpio_interrupt_mask(regs_, pin_mask_, true);
  sg200x_ll_gpio_interrupt_clear(regs_, pin_mask_);
  sg200x_ll_csr_fence_io();
  SG200XGPIO* expected = this;
  (void)instances_[controller][pin_].compare_exchange_strong(
      expected, nullptr, std::memory_order_acq_rel, std::memory_order_acquire);
  regs_ = nullptr;
}

bool SG200XGPIO::Read()
{
  return regs_ != nullptr && (sg200x_ll_gpio_input_get(regs_) & pin_mask_) != 0u;
}

void SG200XGPIO::Write(bool value)
{
  if (regs_ == nullptr) return;
  ConfigurePinmux(pinmux_offset_);
  if (direction_ == Direction::OUTPUT_OPEN_DRAIN && value)
  {
    // Release the output before changing its latch to high.
    sg200x_ll_gpio_output_enable(regs_, pin_mask_, false);
    sg200x_ll_csr_fence_io();
    sg200x_ll_gpio_output_write(regs_, pin_mask_, true);
    return;
  }
  if (direction_ == Direction::OUTPUT_OPEN_DRAIN)
  {
    sg200x_ll_gpio_output_write(regs_, pin_mask_, false);
    sg200x_ll_csr_fence_io();
    sg200x_ll_gpio_output_enable(regs_, pin_mask_, true);
    return;
  }
  if (direction_ == Direction::OUTPUT_PUSH_PULL)
  {
    sg200x_ll_gpio_output_enable(regs_, pin_mask_, true);
  }
  sg200x_ll_gpio_output_write(regs_, pin_mask_, value);
}

ErrorCode SG200XGPIO::SetConfig(Configuration config)
{
  if (regs_ == nullptr)
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

  const uint8_t controller = ControllerIndex(regs_);
  if (controller >= CONTROLLER_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }

  ConfigurePinmux(pinmux_offset_);

  direction_ = config.direction;
  interrupt_enabled_.store(false, std::memory_order_release);
  sg200x_ll_csr_fence_io();
  sg200x_ll_gpio_interrupt_enable(regs_, pin_mask_, false);
  sg200x_ll_gpio_interrupt_mask(regs_, pin_mask_, true);
  sg200x_ll_gpio_interrupt_clear(regs_, pin_mask_);

  switch (config.direction)
  {
    case Direction::INPUT:
      sg200x_ll_gpio_output_enable(regs_, pin_mask_, false);
      sg200x_ll_gpio_interrupt_edge_set(regs_, pin_mask_, false);
      break;
    case Direction::OUTPUT_PUSH_PULL:
      sg200x_ll_gpio_output_enable(regs_, pin_mask_, true);
      sg200x_ll_gpio_interrupt_edge_set(regs_, pin_mask_, false);
      break;
    case Direction::OUTPUT_OPEN_DRAIN:
      sg200x_ll_gpio_output_write(regs_, pin_mask_, false);
      sg200x_ll_csr_fence_io();
      sg200x_ll_gpio_output_enable(regs_, pin_mask_, true);
      sg200x_ll_gpio_interrupt_edge_set(regs_, pin_mask_, false);
      break;
    case Direction::RISING_INTERRUPT:
      sg200x_ll_gpio_output_enable(regs_, pin_mask_, false);
      sg200x_ll_gpio_interrupt_edge_set(regs_, pin_mask_, true);
      sg200x_ll_gpio_interrupt_polarity_set(regs_, pin_mask_, true);
      break;
    case Direction::FALL_INTERRUPT:
      sg200x_ll_gpio_output_enable(regs_, pin_mask_, false);
      sg200x_ll_gpio_interrupt_edge_set(regs_, pin_mask_, true);
      sg200x_ll_gpio_interrupt_polarity_set(regs_, pin_mask_, false);
      break;
    case Direction::FALL_RISING_INTERRUPT:
      return ErrorCode::NOT_SUPPORT;
  }

  return ErrorCode::OK;
}

ErrorCode SG200XGPIO::EnableInterrupt()
{
  if (regs_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  if (direction_ != Direction::RISING_INTERRUPT &&
      direction_ != Direction::FALL_INTERRUPT)
  {
    return ErrorCode::STATE_ERR;
  }

  const uint8_t controller = ControllerIndex(regs_);
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
            expected, IrqRegistrationState::INITIALIZING, std::memory_order_acq_rel,
            std::memory_order_acquire))
    {
      return ErrorCode::BUSY;
    }
    if (request_irq == nullptr ||
        request_irq(irq_, &SG200XGPIO::InterruptHandler, 0u, "sg200x-gpio", nullptr) != 0)
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
  sg200x_ll_csr_fence_io();
  sg200x_ll_gpio_interrupt_clear(regs_, pin_mask_);
  sg200x_ll_gpio_interrupt_mask(regs_, pin_mask_, false);
  sg200x_ll_gpio_interrupt_enable(regs_, pin_mask_, true);
  sg200x_ll_csr_fence_io();
  return ErrorCode::OK;
}

ErrorCode SG200XGPIO::DisableInterrupt()
{
  if (regs_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }

  interrupt_enabled_.store(false, std::memory_order_release);
  sg200x_ll_csr_fence_io();
  sg200x_ll_gpio_interrupt_enable(regs_, pin_mask_, false);
  sg200x_ll_gpio_interrupt_mask(regs_, pin_mask_, true);
  sg200x_ll_gpio_interrupt_clear(regs_, pin_mask_);
  sg200x_ll_csr_fence_io();
  return ErrorCode::OK;
}

void SG200XGPIO::CheckInterrupt(uintptr_t gpio_base)
{
  const uint8_t controller =
      ControllerIndex(reinterpret_cast<const GPIO_Type*>(gpio_base));
  if (controller >= CONTROLLER_COUNT)
  {
    return;
  }

  GPIO_Type* const gpio = sg200x_ll_gpio_get(controller);
  const uint32_t pending = sg200x_ll_gpio_interrupt_status_get(gpio);
  if (pending != 0u)
  {
    sg200x_ll_gpio_interrupt_clear(gpio, pending);
  }

  for (uint8_t pin = 0u; pin < PIN_COUNT; ++pin)
  {
    SG200XGPIO* const instance =
        instances_[controller][pin].load(std::memory_order_acquire);
    if ((pending & (static_cast<uint32_t>(1u) << pin)) != 0u && instance != nullptr &&
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
  CheckInterrupt(reinterpret_cast<uintptr_t>(sg200x_ll_gpio_get(controller)));
  return 0;
}

}  // namespace LibXR

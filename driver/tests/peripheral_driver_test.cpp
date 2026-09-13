#include <sys/mman.h>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "sg200x_adc.hpp"
#include "sg200x_gpio.hpp"
#include "sg200x_i2c.hpp"
#include "sg200x_pwm.hpp"
#include "sg200x_spi.hpp"
#include "sg200x_watchdog.hpp"
#include "sg200x_ll.h"

using namespace LibXR;
using Status = Operation<ErrorCode>::OperationPollingStatus;

// The device line-size constant is 32-bit; the mask must still retain all
// address-size bits when used by the C++ DMA range policy.
static_assert(detail::AlignUpToCacheLine((size_t{1u} << 32u) + 1u) ==
              (size_t{1u} << 32u) + 64u);

namespace
{
using IrqHandler = int (*)(int, void*);
std::array<IrqHandler, NUM_IRQ> handlers{};
uint32_t cache_operations = 0u;

void Map(uintptr_t address)
{
  assert(mmap(reinterpret_cast<void*>(address), 4096u, PROT_READ | PROT_WRITE,
              MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1,
              0) == reinterpret_cast<void*>(address));
}

void Seed(const volatile uint32_t* reg, uint32_t value)
{
  *const_cast<volatile uint32_t*>(reg) = value;
}

DMA_LLI_Type* Descriptor(uint8_t channel)
{
  const auto* ch = sg200x_ll_dma_channel_get(channel);
  const uintptr_t address = ch->LLP_LOW | (uint64_t{ch->LLP_HIGH} << 32u);
  assert(address != 0u && address % DMA_LLI_ALIGNMENT == 0u);
  return reinterpret_cast<DMA_LLI_Type*>(address);
}

void DmaInterrupt(uint8_t channel, bool complete = true)
{
  auto* ch = sg200x_ll_dma_channel_get(channel);
  ch->INTSTATUS = complete ? DMA_INT_TRANSFER_DONE_BIT : DMA_INT_BLOCK_DONE_BIT;
  if (complete) DMA->GLOBAL.CHEN = DMA->GLOBAL.CHEN & ~(1u << channel);
  assert(handlers[IRQ_SDMA] != nullptr);
  handlers[IRQ_SDMA](IRQ_SDMA, nullptr);
  // Apply the hardware W1C side effect to the anonymous register image.
  ch->INTSTATUS = 0u;
}

void I2cInterrupt(uint32_t status)
{
  I2C0_REGS->RAW_INTR_STAT = status;
  assert(handlers[IRQ_I2C0] != nullptr);
  handlers[IRQ_I2C0](IRQ_I2C0, nullptr);
  I2C0_REGS->RAW_INTR_STAT = 0u;
}

void CheckGpio()
{
  using Direction = GPIO::Direction;
  GPIO0_REGS->SWPORTA_DR = 1u;
  GPIO0_REGS->SWPORTA_DDR = 1u;
  unsigned callbacks = 0u;
  {
    SG200XGPIO pin(SG200XGPIO::Bank::A, 14u);
    SG200XGPIO duplicate(SG200XGPIO::Bank::A, 14u);
    assert(duplicate.SetConfig({Direction::INPUT, GPIO::Pull::NONE}) ==
           ErrorCode::ARG_ERR);
    assert(pin.SetConfig({Direction::OUTPUT_OPEN_DRAIN, GPIO::Pull::NONE}) ==
           ErrorCode::OK);
    assert(GPIO0_REGS->SWPORTA_DR == 1u && GPIO0_REGS->SWPORTA_DDR == 0x4001u);
    pin.Write(true);
    assert(GPIO0_REGS->SWPORTA_DDR == 1u && GPIO0_REGS->SWPORTA_DR == 0x4001u);
    pin.Write(false);
    assert(GPIO0_REGS->SWPORTA_DDR == 0x4001u && GPIO0_REGS->SWPORTA_DR == 1u);
    assert(sg200x_ll_pinmux_function_get(PINMUX_SD0_PWR_EN_OFFSET) == 3u);
    assert(pin.SetConfig({Direction::RISING_INTERRUPT, GPIO::Pull::NONE}) ==
           ErrorCode::OK);
    auto callback = GPIO::Callback::Create(
        [](bool in_isr, unsigned* calls)
        {
          assert(in_isr);
          ++*calls;
        },
        &callbacks);
    pin.RegisterCallback(callback);
    assert(pin.EnableInterrupt() == ErrorCode::OK);
    Seed(&GPIO0_REGS->EXT_PORTA, 1u << 14u);
    assert(pin.Read());
    Seed(&GPIO0_REGS->INTSTATUS, 1u << 14u);
    handlers[IRQ_GPIO0](IRQ_GPIO0, nullptr);
    assert(callbacks == 1u && GPIO0_REGS->PORTA_EOI == (1u << 14u));
    assert(pin.DisableInterrupt() == ErrorCode::OK);
    handlers[IRQ_GPIO0](IRQ_GPIO0, nullptr);
    assert(callbacks == 1u);
  }
  SG200XGPIO replacement(SG200XGPIO::Bank::A, 14u);
  assert(replacement.SetConfig({Direction::INPUT, GPIO::Pull::NONE}) == ErrorCode::OK);
  assert(replacement.SetConfig({Direction::INPUT, GPIO::Pull::UP}) ==
         ErrorCode::NOT_SUPPORT);
}

void CheckPwm()
{
  // The caller-supplied 75 MHz must remain the arithmetic input, even though
  // the clock-tree defaults use a different PWM source rate.
  PWM2_REGS->PWMSTART = 8u;
  PWM2_REGS->PWM_OE = 8u;
  SG200XPWM pwm(9u, 75000000u);
  assert(pwm.IsValid() && pwm.SetConfig({20000u}) == ErrorCode::OK);
  assert(pwm.PeriodTicks() == 3750u && pwm.ClockHz() == 75000000u);
  assert(pwm.SetDutyCycle(0.25f) == ErrorCode::OK);
  assert(PWM2_REGS->CHANNEL[1].HLPERIOD == 2812u);
  assert(pwm.SetPolarity(false) == ErrorCode::OK && pwm.Enable() == ErrorCode::OK);
  assert(PWM2_REGS->PWMSTART == 10u && PWM2_REGS->PWM_OE == 10u);
  assert(pwm.SetPolarity(true) == ErrorCode::BUSY);
  assert(pwm.SetConfig({10000u}) == ErrorCode::OK);
  assert(PWM2_REGS->CHANNEL[1].PERIOD == 7500u &&
         PWM2_REGS->CHANNEL[1].HLPERIOD == 5625u);
  assert(pwm.SetConfig({UINT32_MAX}) == ErrorCode::NOT_SUPPORT);
  assert(pwm.Disable() == ErrorCode::OK);
  assert(PWM2_REGS->PWMSTART == 8u && PWM2_REGS->PWM_OE == 8u);
  SG200XPWM invalid(16u);
  assert(!invalid.IsValid());
}

void CheckAdc()
{
  SG200XADC adc({1u, 3u, 4u, 6u}, {3.3f, SG200XADC::Reference::INTERNAL, 15u, 8u});
  assert(adc.IsValid() && adc.ChannelCount() == 4u);
  Seed(&SARADC_REGS->RESULT[0], 0x8FFFu);
  Seed(&SARADC_REGS->RESULT[2], 0x8123u);
  Seed(&RTC_SARADC_REGS->RESULT[0], 0x8456u);
  Seed(&RTC_SARADC_REGS->RESULT[2], 0x8789u);
  const std::array<uint16_t, 4> expected{4095u, 0x123u, 0x456u, 0x789u};
  for (uint8_t index = 0u; index < expected.size(); ++index)
  {
    uint16_t raw = 0u;
    assert(adc.ReadRaw(index, raw) == ErrorCode::OK && raw == expected[index]);
  }
  assert(std::abs(adc.ReadChannel(0u) - 3.3f) < 0.00001f);
  uint16_t untouched = 0xAAAAu;
  Seed(&SARADC_REGS->STATUS, 1u);
  assert(adc.ReadRaw(0u, untouched) == ErrorCode::TIMEOUT && untouched == 0xAAAAu);
  Seed(&SARADC_REGS->STATUS, 0u);
  Seed(&SARADC_REGS->RESULT[0], 1u);
  assert(adc.ReadRaw(0u, untouched) == ErrorCode::FAILED && untouched == 0xAAAAu);
  assert(adc.ReadRaw(4u, untouched) == ErrorCode::ARG_ERR);
  SG200XADC duplicate({1u, 1u});
  assert(!duplicate.IsValid());
}

void CheckWatchdog()
{
  SG200XWatchdog wdt;
  assert(wdt.IsValid() && wdt.IsRunning());
  assert(wdt.TimeoutTop() == 9u && wdt.ActualTimeoutMs() == 1343u);
  assert(WDT2_REGS->TORR == 0x99u && WDT2_REGS->CR == 1u && WDT2_REGS->CRR == 0x76u);
  assert((TOP->WDT_CTRL & 0x44u) == 0x40u);
  assert(wdt.Stop() == ErrorCode::NOT_SUPPORT && !wdt.auto_feed_ && wdt.IsRunning());
  assert(wdt.SetConfig({2000u, 500u}) == ErrorCode::BUSY);
  assert(wdt.Feed() == ErrorCode::OK && WDT2_REGS->CR == 1u);
  SG200XWatchdog slow(SG200XWatchdog::Instance::WDT0, 1000u, 250u, 32768u,
                      SG200XWatchdog::ResetTarget::SYSTEM);
  assert(slow.IsValid() && slow.TimeoutTop() == 0u && slow.ActualTimeoutMs() == 2000u);
  assert((TOP->WDT_CTRL & 0x747u) == 0x141u);
}

void CheckI2c()
{
  auto& rcc = SG200XRCC::Instance();
  assert(rcc.SetRate(SG200XRCC::ClockId::I2c, 50000000u, SG200XRCC::RatePolicy::Exact) ==
         ErrorCode::OK);
  alignas(64) std::array<uint16_t, 64> commands{};
  alignas(64) std::array<uint16_t, 64> receive{};
  SG200XI2C i2c(static_cast<SG200XI2C::Controller>(0u),
                {commands.data(), sizeof(commands)}, {receive.data(), sizeof(receive)},
                {400000u});
  assert(i2c.IsValid());
  assert(I2C0_REGS->SS_SCL_HCNT == 212u && I2C0_REGS->FS_SCL_LCNT == 79u);
  Status status = Status::READY;
  ReadOperation operation(status);
  std::array<uint8_t, 2> data{};
  assert(i2c.MemRead(0x2ABu, 0x1234u, {data.data(), data.size()}, operation,
                     I2C::MemAddrLength::BYTE_16) == ErrorCode::OK);
  assert(status == Status::RUNNING);
  assert(commands[0] == 0x12u && commands[1] == 0x34u && commands[2] == 0x500u &&
         commands[3] == 0x300u);
  assert(I2C0_REGS->TAR == 0x12ABu);
  assert(Descriptor(6u)->block_ts == 3u && Descriptor(7u)->block_ts == 1u);
  receive[0] = 0xABu;
  receive[1] = 0xCDu;
  DmaInterrupt(7u);
  I2cInterrupt(I2C_INTR_STOP_DET_BIT);
  assert(status == Status::RUNNING);
  DmaInterrupt(6u);
  assert(status == Status::DONE && data[0] == 0xABu && data[1] == 0xCDu);

  assert(i2c.Read(0x42u, {data.data(), data.size()}, operation) == ErrorCode::OK);
  assert(commands[0] == 0x100u && commands[1] == 0x300u && I2C0_REGS->TAR == 0x42u);
  I2cInterrupt(I2C_INTR_TX_ABRT_BIT);
  assert(status == Status::ERROR && I2C0_REGS->DMA_CR == 0u);
  assert(i2c.Read(0x42u, {data.data(), data.size()}, operation) == ErrorCode::OK);
  DmaInterrupt(6u);
  DmaInterrupt(7u);
  assert(status == Status::RUNNING);
  I2cInterrupt(I2C_INTR_STOP_DET_BIT);
  assert(status == Status::DONE);
}

void CheckSpi()
{
  alignas(64) std::array<uint8_t, 128> rx{};
  alignas(64) std::array<uint8_t, 128> tx{};
  SG200XSPI spi(static_cast<SG200XSPI::Controller>(2u), 0u, {rx.data(), rx.size()},
                {tx.data(), tx.size()});
  assert(spi.IsValid());
  std::array<uint8_t, 3> output{1u, 2u, 3u};
  std::array<uint8_t, 3> input{};
  Status status = Status::READY;
  WriteOperation operation(status);
  assert(spi.ReadAndWrite({input.data(), input.size()}, {output.data(), output.size()},
                          operation) == ErrorCode::OK);
  const auto* rx_descriptor = Descriptor(4u);
  const auto* tx_descriptor = Descriptor(5u);
  assert(rx_descriptor->block_ts == 2u && tx_descriptor->block_ts == 2u);
  assert(rx_descriptor->next == 0u && tx_descriptor->next == 0u);
  assert(std::memcmp(reinterpret_cast<const void*>(tx_descriptor->source), output.data(),
                     output.size()) == 0);
  const std::array<uint8_t, 3> incoming{4u, 5u, 6u};
  std::memcpy(reinterpret_cast<void*>(rx_descriptor->destination), incoming.data(),
              incoming.size());
  assert((RTC_GPIO_REGS->SWPORTA_DR & PINMUX_SD1_D3_GPIO_MASK) == 0u);
  DmaInterrupt(4u);
  assert(status == Status::DONE && input == incoming);
  assert((RTC_GPIO_REGS->SWPORTA_DR & PINMUX_SD1_D3_GPIO_MASK) != 0u);
  assert(SPI2_REGS->SPIENR == 0u && SPI2_REGS->DMACR == 0u);

  unsigned callbacks = 0u;
  auto callback = WriteOperation::Callback::Create(
      [](bool in_isr, unsigned* calls, ErrorCode result)
      {
        assert(in_isr && result == ErrorCode::OK);
        ++*calls;
      },
      &callbacks);
  WriteOperation circular(callback);
  assert(spi.StartCircularTransfer(8u, circular) == ErrorCode::OK);
  assert(Descriptor(4u)->next == reinterpret_cast<uintptr_t>(Descriptor(4u)));
  assert(Descriptor(5u)->next == reinterpret_cast<uintptr_t>(Descriptor(5u)));
  DmaInterrupt(4u, false);
  DmaInterrupt(4u, false);
  assert(callbacks == 2u);
  assert(spi.StopCircularTransfer() == ErrorCode::OK);
  assert(SG200XDMAC::AcquireFixed(4u) == ErrorCode::OK);
  assert(SG200XDMAC::AcquireFixed(5u) == ErrorCode::OK);
  assert(SG200XDMAC::Release(4u) == ErrorCode::OK &&
         SG200XDMAC::Release(5u) == ErrorCode::OK);
}
}  // namespace

extern "C" int request_irq(unsigned int irq, IrqHandler handler, unsigned long,
                           const char*, void*)
{
  assert(irq < handlers.size());
  handlers[irq] = handler;
  return 0;
}

extern "C" void sg200x_ll_csr_dcache_clean_range(uintptr_t, size_t) { ++cache_operations; }
extern "C" void sg200x_ll_csr_dcache_invalidate_range(uintptr_t, size_t)
{
  ++cache_operations;
}
extern "C" void sg200x_ll_csr_dcache_clean_invalidate_range(uintptr_t, size_t)
{
  ++cache_operations;
}
extern "C" void sg200x_ll_csr_delay_nops(uint32_t) {}

extern "C" bool __real_sg200x_ll_i2c_enable_wait(I2C_Type*, bool, uint32_t);
extern "C" bool __wrap_sg200x_ll_i2c_enable_wait(I2C_Type* i2c, bool enabled,
                                            uint32_t attempts)
{
  // Emulate the hardware enable acknowledgment; retain the real LL request
  // and polling implementation. Timeout behavior is tested by SG200X LL itself.
  if (i2c != nullptr) Seed(&i2c->ENABLE_STATUS, enabled ? 1u : 0u);
  return __real_sg200x_ll_i2c_enable_wait(i2c, enabled, attempts);
}

int main()
{
  for (uintptr_t address :
       {TOP_MISC_BASE, PINMUX_BASE, CLKGEN_BASE, RSTGEN_BASE, GPIO0_BASE, RTC_GPIO_BASE,
        PWM2_BASE, SARADC_BASE, RTC_SARADC_BASE, RTC_CTRL_BASE, WDT0_BASE, WDT2_BASE,
        I2C0_BASE, SPI2_BASE, DMA_BASE, PLIC_BASE_ADDRESS + PLIC_THRESHOLD_OFFSET})
    Map(address);
  PLL_G6->FPLL = (1u << PLL_G6_PREDIV_SHIFT) | (1u << PLL_G6_POSTDIV_SHIFT) |
                 (60u << PLL_G6_MULTIPLIER_SHIFT);
  CheckGpio();
  CheckPwm();
  CheckAdc();
  CheckWatchdog();
  CheckI2c();
  CheckSpi();
  assert(cache_operations != 0u);
  std::puts("SG200X LL-backed GPIO/PWM/ADC/WDT/I2C/SPI/DMA driver contracts passed");
}

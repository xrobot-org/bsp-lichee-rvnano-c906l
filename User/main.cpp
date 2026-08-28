#include "main.h"

#include <atomic>
#include <cstdint>

#include "sg200x_dma.hpp"
#include "sg200x_adc.hpp"
#include "sg200x_gpio.hpp"
#include "sg200x_i2c.hpp"
#include "sg200x_mmio.hpp"
#include "sg200x_pwm.hpp"
#include "sg200x_rcc.hpp"
#include "sg200x_spi.hpp"
#include "sg200x_timebase.hpp"
#include "sg200x_watchdog.hpp"
#include "timer.hpp"

extern "C"
{
#include "arch_helpers.h"
}

namespace
{
// The stock LicheeRV Nano remoteproc driver requires a valid resource-table
// section even when this firmware only runs the LED task.
struct ResourceTable
{
  uint32_t version;
  uint32_t num;
  uint32_t reserved[2];
};

[[gnu::used, gnu::section(".resource_table"), gnu::aligned(8)]]
const ResourceTable RESOURCE_TABLE{1u, 0u, {0u, 0u}};

constexpr uint32_t LED_PERIOD_MS = 1000u;
constexpr uintptr_t BOOT_TRACE_ADDRESS = 0x8FFFF000u;

#ifdef SG200X_DMA_TEST
constexpr uint32_t DMA_TEST_BYTES = 256u;
constexpr uint32_t DMA_TEST_TRACE_EVENT = 1u;  // CVITEK_BOOT_TRACE_EVENT_IRQ_RESULT
constexpr uintptr_t DMA_TEST_TOP_DMA_INT_MUX = 0x03000298u;
constexpr uintptr_t DMA_TEST_PLIC_BASE = 0x70000000u;
constexpr uintptr_t DMA_TEST_PLIC_PENDING1 = DMA_TEST_PLIC_BASE + 0x1000u;
constexpr uintptr_t DMA_TEST_PLIC_ENABLE1 = DMA_TEST_PLIC_BASE + 0x2000u;
constexpr uintptr_t DMA_TEST_PLIC_THRESHOLD = DMA_TEST_PLIC_BASE + 0x200000u;
constexpr uint32_t DMA_TEST_CHANNEL_BASE = 0x100u;
constexpr uint32_t DMA_TEST_CHANNEL_STRIDE = 0x100u;
constexpr uint32_t DMA_TEST_DMAC_INTSTATUS = 0x30u;
constexpr uint32_t DMA_TEST_CH_INTSTATUS_EN = 0x80u;
constexpr uint32_t DMA_TEST_CH_INTSTATUS = 0x88u;
constexpr uint32_t DMA_TEST_CH_INTSIGNAL_EN = 0x90u;
constexpr uint32_t DMA_TEST_TRACE_DIAGNOSTIC_WORD = 40u;
constexpr uint32_t DMA_TEST_TRACE_DIAGNOSTIC_MAGIC = 0x444D4131u;  // "DMA1"
alignas(64) uint8_t dma_test_source[DMA_TEST_BYTES];
alignas(64) uint8_t dma_test_destination[DMA_TEST_BYTES];
volatile bool dma_test_finished = false;
uint8_t dma_test_channel = 0xFFu;
LibXR::Timer::TimerHandle dma_start_timer = nullptr;

uint32_t ReadDmaTestMstatus()
{
  uintptr_t value = 0u;
  asm volatile("csrr %0, mstatus" : "=r"(value) :: "memory");
  return static_cast<uint32_t>(value);
}

uint32_t ReadDmaTestMie()
{
  uintptr_t value = 0u;
  asm volatile("csrr %0, mie" : "=r"(value) :: "memory");
  return static_cast<uint32_t>(value);
}

uint32_t ReadDmaTestMip()
{
  uintptr_t value = 0u;
  asm volatile("csrr %0, mip" : "=r"(value) :: "memory");
  return static_cast<uint32_t>(value);
}

uint32_t FirstPendingDmaTestPlicSource()
{
  // The C906L FreeRTOS port exposes 62 PLIC sources.  The DMA timeout has
  // already proved that a controller event occurred; finding any PLIC source
  // here distinguishes a wrong source number from a missing TOP-to-PLIC
  // route.  Source 63 is outside the port's valid range and is our sentinel.
  for (uint32_t word = 0u; word < 2u; ++word)
  {
    uint32_t pending = LibXR::Register32(DMA_TEST_PLIC_PENDING1 + word * 4u);
    for (uint32_t bit = 0u; bit < 32u; ++bit)
    {
      if ((pending & (1u << bit)) != 0u)
      {
        return word * 32u + bit;
      }
    }
  }
  return 63u;
}

uint32_t PackDmaTestInterruptState(uint8_t channel)
{
  if (channel >= LibXR::SG200XDMAC::CHANNEL_COUNT)
  {
    return 0x00FFFFFFu;
  }

  const uintptr_t channel_base = LibXR::SG200XDMAC::BASE + DMA_TEST_CHANNEL_BASE +
                                 static_cast<uintptr_t>(channel) * DMA_TEST_CHANNEL_STRIDE;
  const uint32_t config = LibXR::Register32(LibXR::SG200XDMAC::BASE + 0x10u);
  const uint32_t global_status =
      LibXR::Register32(LibXR::SG200XDMAC::BASE + DMA_TEST_DMAC_INTSTATUS);
  const uint32_t channel_status = LibXR::Register32(channel_base + DMA_TEST_CH_INTSTATUS);

  // The versioned Linux boot-trace ABI exposes 24 event-detail bits.  Keep
  // the timeout result self-contained when the raw diagnostic extension is
  // unavailable through a no-map reserved-memory mapping.  The top six bits
  // identify a pending C906L PLIC source; 63 means none of its 62 sources
  // latched while the DMA controller held its completion status.
  return (config & 0x3u) | ((global_status & 0xFFu) << 2u) |
         ((channel_status & 0xFFu) << 10u) |
         ((FirstPendingDmaTestPlicSource() & 0x3Fu) << 18u);
}

void PublishDmaTestDiagnostics(uint8_t channel)
{
  if (channel >= LibXR::SG200XDMAC::CHANNEL_COUNT)
  {
    return;
  }

  const uintptr_t channel_base = LibXR::SG200XDMAC::BASE + DMA_TEST_CHANNEL_BASE +
                                 static_cast<uintptr_t>(channel) * DMA_TEST_CHANNEL_STRIDE;
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  // These words are outside the Linux driver's versioned boot-trace struct.
  // They are only consumed by the DMA self-test collector after a timeout.
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 0u] = DMA_TEST_TRACE_DIAGNOSTIC_MAGIC;
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 1u] =
       LibXR::Register32(DMA_TEST_TOP_DMA_INT_MUX);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 2u] =
       LibXR::Register32(LibXR::SG200XDMAC::BASE + 0x10u);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 3u] =
       LibXR::Register32(LibXR::SG200XDMAC::BASE + DMA_TEST_DMAC_INTSTATUS);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 4u] =
       LibXR::Register32(channel_base + DMA_TEST_CH_INTSTATUS);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 5u] =
       LibXR::Register32(channel_base + DMA_TEST_CH_INTSTATUS_EN);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 6u] =
       LibXR::Register32(channel_base + DMA_TEST_CH_INTSIGNAL_EN);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 7u] = LibXR::Register32(DMA_TEST_PLIC_PENDING1);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 8u] = LibXR::Register32(DMA_TEST_PLIC_ENABLE1);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 9u] =
       LibXR::Register32(DMA_TEST_PLIC_BASE + LibXR::SG200XDMAC::IRQ * 4u);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 10u] =
       LibXR::Register32(DMA_TEST_PLIC_THRESHOLD);
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 11u] = ReadDmaTestMstatus();
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 12u] = ReadDmaTestMie();
  trace[DMA_TEST_TRACE_DIAGNOSTIC_WORD + 13u] = ReadDmaTestMip();
  flush_dcache_range(BOOT_TRACE_ADDRESS + DMA_TEST_TRACE_DIAGNOSTIC_WORD * sizeof(uint32_t),
                     14u * sizeof(uint32_t));
}

void PublishDmaTestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  // The Linux C906L driver decodes event_sequence/event/event_arg in the
  // second half of the first cache line.  Keep the result self-describing:
  // the high byte is the result code and the low 24 bits carry detail.
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (DMA_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = DMA_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void DmaTestComplete(void*, LibXR::ErrorCode result, bool)
{
  uint32_t mismatch = DMA_TEST_BYTES;
  if (result == LibXR::ErrorCode::OK)
  {
    for (uint32_t index = 0u; index < DMA_TEST_BYTES; ++index)
    {
      if (dma_test_source[index] != dma_test_destination[index])
      {
        mismatch = index;
        break;
      }
    }
  }
  PublishDmaTestResult(
      result == LibXR::ErrorCode::OK && mismatch == DMA_TEST_BYTES ? 0u : 1u, mismatch);
  LibXR::SG200XDMAC::Release(dma_test_channel);
  dma_test_channel = 0xFFu;
  dma_test_finished = true;
}

void DmaTestTimeout(void*)
{
  if (!dma_test_finished)
  {
    // Capture latched DMAC status before Release() acknowledges it.  This
    // tells board validation whether a callback timeout is a DMA, routing,
    // PLIC, or machine-external-interrupt enable failure.
    const uint32_t interrupt_state =
        PackDmaTestInterruptState(dma_test_channel);
    PublishDmaTestDiagnostics(dma_test_channel);
    LibXR::SG200XDMAC::Release(dma_test_channel);
    dma_test_channel = 0xFFu;
    // 2 = callback timeout; the detail packs the interrupt-route snapshot.
    PublishDmaTestResult(2u, interrupt_state);
    dma_test_finished = true;
  }
}

void StartDmaTest();

void StartDmaTestDeferred(void*)
{
  if (dma_start_timer != nullptr)
  {
    LibXR::Timer::Stop(dma_start_timer);
  }
  StartDmaTest();
}

void StartDmaTest()
{
  // Publish an early marker so a board run distinguishes task startup from a
  // later DMA/IRQ failure.
  PublishDmaTestResult(0xFFFFFFFFu, 0u);
  for (uint32_t index = 0u; index < DMA_TEST_BYTES; ++index)
  {
    dma_test_source[index] = static_cast<uint8_t>((index * 37u) ^ 0xA5u);
    dma_test_destination[index] = 0u;
  }

  uint8_t channel = 0xFFu;
  const LibXR::ErrorCode acquire_result = LibXR::SG200XDMAC::Acquire(channel);
  if (acquire_result != LibXR::ErrorCode::OK)
  {
    PublishDmaTestResult(3u, static_cast<uint32_t>(-static_cast<int8_t>(acquire_result)));
    return;
  }
  dma_test_channel = channel;

  const LibXR::ErrorCode start_result = LibXR::SG200XDMAC::Start(
      channel, {.memory = reinterpret_cast<uintptr_t>(dma_test_source),
                .memory_capacity = sizeof(dma_test_source),
                .peripheral = reinterpret_cast<uintptr_t>(dma_test_destination),
                .peripheral_capacity = sizeof(dma_test_destination),
                .count = DMA_TEST_BYTES,
                .direction = LibXR::SG200XDMAC::Direction::MEMORY_TO_MEMORY,
                .width = LibXR::SG200XDMAC::Width::BYTE,
                .callback = DmaTestComplete,
                .context = nullptr});
  if (start_result != LibXR::ErrorCode::OK)
  {
    LibXR::SG200XDMAC::Release(channel);
    dma_test_channel = 0xFFu;
    PublishDmaTestResult(4u, static_cast<uint32_t>(-static_cast<int8_t>(start_result)));
    return;
  }
  static const auto timeout =
      LibXR::Timer::CreateTask<void*>(DmaTestTimeout, nullptr, 5000u);
  if (timeout != nullptr)
  {
    LibXR::Timer::Add(timeout);
    LibXR::Timer::Start(timeout);
  }
}
#endif

#ifdef SG200X_I2C_DMA_TEST
constexpr uint32_t I2C_TEST_TRACE_EVENT = 1u;  // CVITEK_BOOT_TRACE_EVENT_IRQ_RESULT
alignas(64) uint16_t i2c_test_tx_stage[32]{};
alignas(64) uint16_t i2c_test_rx_stage[32]{};
uint8_t i2c_test_value[2]{};
LibXR::Thread i2c_test_thread;

void PublishI2cTestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (I2C_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = I2C_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void StartI2cDmaTest()
{
  // INA228 MANUFACTURER_ID is a read-only 16-bit register with reset value
  // 0x5449 ("TI"). Reading it exercises the register-address write, repeated
  // START, both DMA streams, STOP/ABRT IRQ, and LibXR BLOCK completion path.
  static LibXR::SG200XI2C i2c(
      LibXR::SG200XI2C::Controller::I2C1,
      {i2c_test_tx_stage, sizeof(i2c_test_tx_stage)},
      {i2c_test_rx_stage, sizeof(i2c_test_rx_stage)}, {100000u});
  if (!i2c.IsValid())
  {
    PublishI2cTestResult(3u, 0u);
    return;
  }
  LibXR::Semaphore semaphore{};
  LibXR::ReadOperation operation(semaphore, 1000u);
  const auto result = i2c.MemRead(
      0x40u, 0x3Eu, {i2c_test_value, sizeof(i2c_test_value)}, operation,
      LibXR::I2C::MemAddrLength::BYTE_8);
  const uint32_t value =
      (static_cast<uint32_t>(i2c_test_value[0]) << 8u) | i2c_test_value[1];
  const uint32_t code = result != LibXR::ErrorCode::OK
                            ? static_cast<uint32_t>(-static_cast<int8_t>(result))
                            : value == 0x5449u ? 0u : 1u;
  PublishI2cTestResult(code, value);
}

void StartI2cDmaTestTask(void*) { StartI2cDmaTest(); }
#endif

#ifdef SG200X_I2C1_INA228_TEST
constexpr uintptr_t INA228_I2C1_SCL_PINMUX = 0xD0u;  // P18 / SD1_D3
constexpr uintptr_t INA228_I2C1_SDA_PINMUX = 0xDCu;  // P21 / SD1_D0
constexpr uint16_t INA228_ADDRESS = 0x40u;
constexpr uint16_t INA228_MANUFACTURER_ID_REGISTER = 0x3Eu;
constexpr uint16_t INA228_MANUFACTURER_ID = 0x5449u;
constexpr uint16_t INA228_DEVICE_ID_REGISTER = 0x3Fu;
constexpr uint16_t INA228_DEVICE_ID = 0x2281u;
constexpr uint16_t INA228_CONFIG_REGISTER = 0x00u;
constexpr uint16_t INA228_UNUSED_ADDRESS = 0x41u;
#ifndef SG200X_I2C1_INA228_STRESS_ITERATIONS
#define SG200X_I2C1_INA228_STRESS_ITERATIONS 4096u
#endif
constexpr uint32_t INA228_STRESS_ITERATIONS =
    SG200X_I2C1_INA228_STRESS_ITERATIONS;
alignas(64) uint16_t ina228_i2c_tx_stage[32]{};
alignas(64) uint16_t ina228_i2c_rx_stage[32]{};
uint8_t ina228_id_bytes[2]{};
LibXR::Thread ina228_i2c_test_thread;
std::atomic<bool> ina228_i2c_test_completed{false};
std::atomic<int8_t> ina228_i2c_test_result{static_cast<int8_t>(LibXR::ErrorCode::OK)};
uint32_t ina228_i2c_timeout_state = 0u;

[[nodiscard]] uint32_t CaptureIna228I2cTimeoutState()
{
  constexpr uintptr_t I2C1_BASE = 0x04010000u;
  constexpr uintptr_t DMAC_BASE = LibXR::SG200XDMAC::BASE;
  constexpr uintptr_t PLIC_PENDING1 = 0x70001000u;
  const uint32_t raw = LibXR::Register32(I2C1_BASE + 0x34u);
  const uint32_t status = LibXR::Register32(I2C1_BASE + 0x70u);
  const uint32_t dma_enabled = LibXR::Register32(DMAC_BASE + 0x18u);
  const uint32_t dma_interrupt = LibXR::Register32(DMAC_BASE + 0x30u);
  const uint32_t plic_low = LibXR::Register32(PLIC_PENDING1);
  const uint32_t plic_high = LibXR::Register32(PLIC_PENDING1 + 4u);
  uintptr_t mip = 0u;
  asm volatile("csrr %0, mip" : "=r"(mip) :: "memory");

  return (raw & 0x3FFu) | ((status & 0x7Fu) << 10u) |
         (((dma_enabled >> 4u) & 0x3u) << 17u) |
         (((dma_interrupt >> 4u) & 0x3u) << 19u) |
         (((plic_low >> 25u) & 0x1u) << 21u) |
         (((plic_high >> 1u) & 0x1u) << 22u) |
         ((static_cast<uint32_t>(mip >> 11u) & 0x1u) << 23u);
}

void PublishIna228TestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u;
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = 1u;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void ConfigureIna228I2c1Pinmux()
{
  constexpr uintptr_t PINMUX_BASE = 0x03001000u;
  LibXR::Register32(PINMUX_BASE, INA228_I2C1_SCL_PINMUX) =
      (LibXR::Register32(PINMUX_BASE, INA228_I2C1_SCL_PINMUX) & ~0x7u) | 2u;
  LibXR::Register32(PINMUX_BASE, INA228_I2C1_SDA_PINMUX) =
      (LibXR::Register32(PINMUX_BASE, INA228_I2C1_SDA_PINMUX) & ~0x7u) | 2u;
}

void PublishIna228Failure(uint32_t stage, LibXR::ErrorCode result,
                          uint16_t detail = 0u)
{
  const uint32_t error = static_cast<uint8_t>(-static_cast<int8_t>(result));
  PublishIna228TestResult(stage, (error << 16u) | detail);
}

[[nodiscard]] uint16_t Ina228ReadValue()
{
  return static_cast<uint16_t>(static_cast<uint16_t>(ina228_id_bytes[0]) << 8u |
                               ina228_id_bytes[1]);
}

void Ina228I2c1TestComplete(bool, void*, LibXR::ErrorCode result)
{
  ina228_i2c_test_result.store(static_cast<int8_t>(result), std::memory_order_relaxed);
  ina228_i2c_test_completed.store(true, std::memory_order_release);
}

[[nodiscard]] LibXR::ErrorCode WaitForIna228Callback(uint32_t timeout_us = 100000u)
{
  const uint64_t wait_until = LibXR::Timebase::GetMicroseconds() + timeout_us;
  while (!ina228_i2c_test_completed.load(std::memory_order_acquire) &&
         LibXR::Timebase::GetMicroseconds() < wait_until)
  {
    asm volatile("nop");
  }
  return ina228_i2c_test_completed.load(std::memory_order_acquire)
             ? static_cast<LibXR::ErrorCode>(
                   ina228_i2c_test_result.load(std::memory_order_relaxed))
             : LibXR::ErrorCode::TIMEOUT;
}

[[nodiscard]] LibXR::ErrorCode Ina228CallbackRead(LibXR::SG200XI2C& i2c,
                                                  uint16_t reg)
{
  static auto callback = LibXR::Callback<LibXR::ErrorCode>::Create<void*>(
      Ina228I2c1TestComplete, nullptr);
  LibXR::ReadOperation operation(callback);
  ina228_i2c_test_completed.store(false, std::memory_order_release);
  ina228_i2c_test_result.store(static_cast<int8_t>(LibXR::ErrorCode::PENDING),
                               std::memory_order_relaxed);
  const LibXR::ErrorCode start = i2c.MemRead(
      INA228_ADDRESS, reg, {ina228_id_bytes, sizeof(ina228_id_bytes)}, operation);
  return start == LibXR::ErrorCode::OK ? WaitForIna228Callback() : start;
}

[[gnu::noinline]] LibXR::ErrorCode StartIna228DetachedCallbackRead(
    LibXR::SG200XI2C& i2c, uint16_t reg)
{
  static auto callback = LibXR::Callback<LibXR::ErrorCode>::Create<void*>(
      Ina228I2c1TestComplete, nullptr);
  LibXR::ReadOperation operation(callback);
  return i2c.MemRead(INA228_ADDRESS, reg,
                     {ina228_id_bytes, sizeof(ina228_id_bytes)}, operation);
}

[[gnu::noinline]] void ClobberIna228OperationStack()
{
  volatile uintptr_t scratch[32]{};
  for (size_t index = 0u; index < 32u; ++index)
  {
    scratch[index] = 0xA5A50000u + index;
  }
  asm volatile("" : : "r"(scratch) : "memory");
}

[[nodiscard]] LibXR::ErrorCode Ina228PollingRead(LibXR::SG200XI2C& i2c,
                                                 uint16_t reg)
{
  ina228_i2c_timeout_state = 0u;
  LibXR::ReadOperation::OperationPollingStatus status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation operation(status);
  const LibXR::ErrorCode start = i2c.MemRead(
      INA228_ADDRESS, reg, {ina228_id_bytes, sizeof(ina228_id_bytes)}, operation);
  if (start != LibXR::ErrorCode::OK)
  {
    return start;
  }
  const uint64_t wait_until = LibXR::Timebase::GetMicroseconds() + 100000u;
  while (status == LibXR::ReadOperation::OperationPollingStatus::RUNNING &&
         LibXR::Timebase::GetMicroseconds() < wait_until)
  {
    asm volatile("nop");
  }
  if (status == LibXR::ReadOperation::OperationPollingStatus::DONE)
  {
    return LibXR::ErrorCode::OK;
  }
  if (status == LibXR::ReadOperation::OperationPollingStatus::ERROR)
  {
    return LibXR::ErrorCode::FAILED;
  }
  ina228_i2c_timeout_state = CaptureIna228I2cTimeoutState();
  return LibXR::ErrorCode::TIMEOUT;
}

[[nodiscard]] uintptr_t DisableMachineInterrupts()
{
  constexpr uintptr_t MSTATUS_MIE = 1u << 3u;
  uintptr_t previous = 0u;
  asm volatile("csrrc %0, mstatus, %1" : "=r"(previous) : "r"(MSTATUS_MIE) : "memory");
  return previous;
}

void RestoreMachineInterrupts(uintptr_t previous)
{
  constexpr uintptr_t MSTATUS_MIE = 1u << 3u;
  if ((previous & MSTATUS_MIE) != 0u)
  {
    asm volatile("csrs mstatus, %0" : : "r"(MSTATUS_MIE) : "memory");
  }
}

void StartIna228I2c1Test(void*)
{
  PublishIna228TestResult(0xFFu, 0u);
  ConfigureIna228I2c1Pinmux();
  PublishIna228TestResult(0xFEu, 0u);
  static LibXR::SG200XI2C i2c(
      LibXR::SG200XI2C::Controller::I2C1,
      {ina228_i2c_tx_stage, sizeof(ina228_i2c_tx_stage)},
      {ina228_i2c_rx_stage, sizeof(ina228_i2c_rx_stage)}, {100000u});
  if (!i2c.IsValid())
  {
    PublishIna228Failure(1u, LibXR::ErrorCode::INIT_ERR);
    return;
  }

  static LibXR::Semaphore semaphore{};
  LibXR::ReadOperation block_read(semaphore, 1000u);
  LibXR::ErrorCode result = i2c.SetConfig({100000u});
  if (result != LibXR::ErrorCode::OK)
  {
    PublishIna228Failure(2u, result);
    return;
  }

  result = i2c.MemRead(
      INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
      {ina228_id_bytes, sizeof(ina228_id_bytes)}, block_read);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_MANUFACTURER_ID)
  {
    PublishIna228Failure(3u, result, Ina228ReadValue());
    return;
  }

  result = Ina228CallbackRead(i2c, INA228_DEVICE_ID_REGISTER);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_DEVICE_ID)
  {
    PublishIna228Failure(4u, result, Ina228ReadValue());
    return;
  }

  result = Ina228PollingRead(i2c, INA228_CONFIG_REGISTER);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != 0u)
  {
    PublishIna228Failure(5u, result, Ina228ReadValue());
    return;
  }

  LibXR::ReadOperation::OperationPollingStatus empty_status =
      LibXR::ReadOperation::OperationPollingStatus::READY;
  LibXR::ReadOperation empty_operation(empty_status);
  result = i2c.Read(INA228_ADDRESS, {}, empty_operation);
  if (result != LibXR::ErrorCode::OK ||
      empty_status != LibXR::ReadOperation::OperationPollingStatus::DONE)
  {
    PublishIna228Failure(6u, result, static_cast<uint16_t>(empty_status));
    return;
  }

  uint8_t oversized[33]{};
  LibXR::ReadOperation contract_operation(semaphore, 1000u);
  if (i2c.Read(0x400u, {ina228_id_bytes, 1u}, contract_operation) !=
          LibXR::ErrorCode::ARG_ERR ||
      i2c.Read(INA228_ADDRESS, {nullptr, 1u}, contract_operation) !=
          LibXR::ErrorCode::ARG_ERR ||
      i2c.MemRead(INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
                  {oversized, sizeof(oversized)}, contract_operation) !=
          LibXR::ErrorCode::SIZE_ERR ||
      i2c.MemRead(INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
                  {ina228_id_bytes, sizeof(ina228_id_bytes)}, contract_operation,
                  static_cast<LibXR::I2C::MemAddrLength>(0xFFu)) !=
          LibXR::ErrorCode::ARG_ERR ||
      i2c.Read(INA228_ADDRESS, {ina228_id_bytes, 1u}, contract_operation, true) !=
          LibXR::ErrorCode::NOT_SUPPORT)
  {
    PublishIna228Failure(7u, LibXR::ErrorCode::CHECK_ERR);
    return;
  }

  static auto callback = LibXR::Callback<LibXR::ErrorCode>::Create<void*>(
      Ina228I2c1TestComplete, nullptr);
  LibXR::ReadOperation busy_operation(callback);
  ina228_i2c_test_completed.store(false, std::memory_order_release);
  const uintptr_t saved_mstatus = DisableMachineInterrupts();
  const LibXR::ErrorCode busy_start = i2c.MemRead(
      INA228_ADDRESS, INA228_DEVICE_ID_REGISTER,
      {ina228_id_bytes, sizeof(ina228_id_bytes)}, busy_operation);
  const LibXR::ErrorCode busy_result = i2c.SetConfig({400000u});
  RestoreMachineInterrupts(saved_mstatus);
  result = busy_start == LibXR::ErrorCode::OK ? WaitForIna228Callback() : busy_start;
  if (busy_result != LibXR::ErrorCode::BUSY || result != LibXR::ErrorCode::OK ||
      Ina228ReadValue() != INA228_DEVICE_ID)
  {
    PublishIna228Failure(8u,
                         busy_result != LibXR::ErrorCode::BUSY ? busy_result : result,
                         Ina228ReadValue());
    return;
  }

  uint8_t register_pointer = static_cast<uint8_t>(INA228_MANUFACTURER_ID_REGISTER);
  LibXR::WriteOperation pointer_write(semaphore, 1000u);
  result = i2c.Write(INA228_ADDRESS, {&register_pointer, sizeof(register_pointer)},
                     pointer_write);
  if (result == LibXR::ErrorCode::OK)
  {
    LibXR::ReadOperation pointer_read(semaphore, 1000u);
    result = i2c.Read(INA228_ADDRESS, {ina228_id_bytes, sizeof(ina228_id_bytes)},
                      pointer_read);
  }
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_MANUFACTURER_ID)
  {
    PublishIna228Failure(9u, result, Ina228ReadValue());
    return;
  }

  result = i2c.SetConfig({400000u});
  if (result != LibXR::ErrorCode::OK)
  {
    PublishIna228Failure(10u, result);
    return;
  }
  LibXR::ReadOperation fast_read(semaphore, 1000u);
  result = i2c.MemRead(INA228_ADDRESS, INA228_DEVICE_ID_REGISTER,
                       {ina228_id_bytes, sizeof(ina228_id_bytes)}, fast_read);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_DEVICE_ID)
  {
    PublishIna228Failure(11u, result, Ina228ReadValue());
    return;
  }
  if (i2c.SetConfig({123456u}) != LibXR::ErrorCode::NOT_SUPPORT)
  {
    PublishIna228Failure(12u, LibXR::ErrorCode::CHECK_ERR);
    return;
  }

  LibXR::ReadOperation nack_read(semaphore, 1000u);
  result = i2c.Read(INA228_UNUSED_ADDRESS, {ina228_id_bytes, 1u}, nack_read);
  if (result != LibXR::ErrorCode::NO_RESPONSE || !i2c.IsValid())
  {
    PublishIna228Failure(13u, result);
    return;
  }

  LibXR::ReadOperation recovery_read(semaphore, 1000u);
  result = i2c.MemRead(INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
                       {ina228_id_bytes, sizeof(ina228_id_bytes)}, recovery_read);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_MANUFACTURER_ID)
  {
    PublishIna228Failure(14u, result, Ina228ReadValue());
    return;
  }

  const uint8_t config_value[2] = {0u, 0u};
  LibXR::WriteOperation config_write(semaphore, 1000u);
  result = i2c.MemWrite(INA228_ADDRESS, INA228_CONFIG_REGISTER,
                        {config_value, sizeof(config_value)}, config_write);
  if (result == LibXR::ErrorCode::OK)
  {
    LibXR::ReadOperation config_read(semaphore, 1000u);
    result = i2c.MemRead(INA228_ADDRESS, INA228_CONFIG_REGISTER,
                         {ina228_id_bytes, sizeof(ina228_id_bytes)}, config_read);
  }
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != 0u)
  {
    PublishIna228Failure(15u, result, Ina228ReadValue());
    return;
  }

  ina228_i2c_test_completed.store(false, std::memory_order_release);
  ina228_i2c_test_result.store(static_cast<int8_t>(LibXR::ErrorCode::PENDING),
                               std::memory_order_relaxed);
  const uintptr_t detached_mstatus = DisableMachineInterrupts();
  const LibXR::ErrorCode detached_start =
      StartIna228DetachedCallbackRead(i2c, INA228_DEVICE_ID_REGISTER);
  ClobberIna228OperationStack();
  RestoreMachineInterrupts(detached_mstatus);
  result = detached_start == LibXR::ErrorCode::OK ? WaitForIna228Callback()
                                                   : detached_start;
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_DEVICE_ID)
  {
    PublishIna228Failure(16u, result, Ina228ReadValue());
    return;
  }

  uint8_t held_dma_channels[3] = {0xFFu, 0xFFu, 0xFFu};
  for (uint8_t index = 0u; index < 3u; ++index)
  {
    result = LibXR::SG200XDMAC::Acquire(held_dma_channels[index]);
    if (result != LibXR::ErrorCode::OK)
    {
      for (uint8_t release = 0u; release < index; ++release)
      {
        (void)LibXR::SG200XDMAC::Release(held_dma_channels[release]);
      }
      PublishIna228Failure(17u, result, index);
      return;
    }
  }
  LibXR::ReadOperation dma_busy_read(semaphore, 1000u);
  result = i2c.MemRead(INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
                       {ina228_id_bytes, sizeof(ina228_id_bytes)}, dma_busy_read);
  LibXR::ErrorCode release_result = LibXR::ErrorCode::OK;
  for (uint8_t channel : held_dma_channels)
  {
    const LibXR::ErrorCode channel_result = LibXR::SG200XDMAC::Release(channel);
    if (channel_result != LibXR::ErrorCode::OK)
    {
      release_result = channel_result;
    }
  }
  if (result != LibXR::ErrorCode::BUSY || release_result != LibXR::ErrorCode::OK ||
      !i2c.IsValid())
  {
    PublishIna228Failure(18u,
                         result != LibXR::ErrorCode::BUSY ? result : release_result);
    return;
  }
  LibXR::ReadOperation dma_recovery_read(semaphore, 1000u);
  result = i2c.MemRead(INA228_ADDRESS, INA228_MANUFACTURER_ID_REGISTER,
                       {ina228_id_bytes, sizeof(ina228_id_bytes)}, dma_recovery_read);
  if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != INA228_MANUFACTURER_ID)
  {
    PublishIna228Failure(19u, result, Ina228ReadValue());
    return;
  }

  for (uint32_t iteration = 0u; iteration < INA228_STRESS_ITERATIONS; ++iteration)
  {
    if ((iteration & 0xFFu) == 0u)
    {
      const uint32_t speed = ((iteration >> 8u) & 1u) == 0u ? 100000u : 400000u;
      result = i2c.SetConfig({speed});
      if (result != LibXR::ErrorCode::OK)
      {
        PublishIna228Failure(20u, result, static_cast<uint16_t>(iteration));
        return;
      }
    }

    const bool manufacturer = (iteration & 1u) == 0u;
    const uint16_t reg = manufacturer ? INA228_MANUFACTURER_ID_REGISTER
                                      : INA228_DEVICE_ID_REGISTER;
    const uint16_t expected = manufacturer ? INA228_MANUFACTURER_ID : INA228_DEVICE_ID;
    switch (iteration % 3u)
    {
      case 0u:
      {
        LibXR::ReadOperation operation(semaphore, 1000u);
        result = i2c.MemRead(INA228_ADDRESS, reg,
                             {ina228_id_bytes, sizeof(ina228_id_bytes)}, operation);
        break;
      }
      case 1u:
        result = Ina228CallbackRead(i2c, reg);
        break;
      default:
        result = Ina228PollingRead(i2c, reg);
        break;
    }
    if (result != LibXR::ErrorCode::OK || Ina228ReadValue() != expected)
    {
      if (result == LibXR::ErrorCode::TIMEOUT)
      {
        PublishIna228TestResult(22u, ina228_i2c_timeout_state);
        return;
      }
      PublishIna228Failure(21u, result, static_cast<uint16_t>(iteration));
      return;
    }
  }
  PublishIna228TestResult(0u, INA228_STRESS_ITERATIONS);
}
#endif

#ifdef SG200X_SPI4_TEST
constexpr uint32_t SPI4_TEST_TRACE_EVENT = 1u;
constexpr uint8_t SPI4_TEST_BYTES[] = {0x00u, 0xFFu, 0xA5u, 0x5Au,
                                       0x3Cu, 0xC3u, 0x81u, 0x18u};

void PublishSpi4TestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (SPI4_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = SPI4_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void StartSpi4Test(void*)
{
  static LibXR::SG200XGPIO sck(LibXR::SG200XGPIO::Bank::A, 22u);
  static LibXR::SG200XGPIO miso(LibXR::SG200XGPIO::Bank::A, 23u);
  static LibXR::SG200XGPIO cs(LibXR::SG200XGPIO::Bank::A, 24u);
  static LibXR::SG200XGPIO mosi(LibXR::SG200XGPIO::Bank::A, 25u);
  const LibXR::GPIO::Configuration output{LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
                                          LibXR::GPIO::Pull::NONE};
  const LibXR::GPIO::Configuration input{LibXR::GPIO::Direction::INPUT,
                                         LibXR::GPIO::Pull::NONE};
  if (sck.SetConfig(output) != LibXR::ErrorCode::OK ||
      cs.SetConfig(output) != LibXR::ErrorCode::OK ||
      mosi.SetConfig(output) != LibXR::ErrorCode::OK ||
      miso.SetConfig(input) != LibXR::ErrorCode::OK)
  {
    PublishSpi4TestResult(2u, 0u);
    return;
  }

  sck.Write(false);
  cs.Write(true);
  mosi.Write(false);
  LibXR::Timebase::DelayMicroseconds(2u);
  cs.Write(false);
  uint32_t mismatch = sizeof(SPI4_TEST_BYTES);
  for (uint32_t index = 0u; index < sizeof(SPI4_TEST_BYTES); ++index)
  {
    uint8_t received = 0u;
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
      mosi.Write((SPI4_TEST_BYTES[index] & (0x80u >> bit)) != 0u);
      LibXR::Timebase::DelayMicroseconds(1u);
      sck.Write(true);
      LibXR::Timebase::DelayMicroseconds(1u);
      received = static_cast<uint8_t>((received << 1u) | (miso.Read() ? 1u : 0u));
      sck.Write(false);
      LibXR::Timebase::DelayMicroseconds(1u);
    }
    if (mismatch == sizeof(SPI4_TEST_BYTES) && received != SPI4_TEST_BYTES[index])
    {
      mismatch = index;
    }
  }
  cs.Write(true);
  PublishSpi4TestResult(mismatch == sizeof(SPI4_TEST_BYTES) ? 0u : 1u, mismatch);
}
#endif

#if defined(SG200X_SPI2_TEST) || defined(SG200X_SPI2_STRESS_TEST)
constexpr uint32_t SPI2_TEST_TRACE_EVENT = 1u;
constexpr uint32_t SPI2_TEST_BYTES = 64u;
constexpr uint32_t SPI2_TEST_BUFFER_BYTES = 512u;
constexpr uintptr_t SPI2_TEST_PINMUX_SD1_D3 = 0xD0u;  // P18 / CS
constexpr uintptr_t SPI2_TEST_PINMUX_SD1_D0 = 0xDCu;  // P21 / MISO
constexpr uintptr_t SPI2_TEST_PINMUX_SD1_CMD = 0xE0u; // P22 / MOSI
constexpr uintptr_t SPI2_TEST_PINMUX_SD1_CLK = 0xE4u; // P23 / SCK
alignas(64) uint8_t spi2_test_rx_dma[SPI2_TEST_BUFFER_BYTES]{};
alignas(64) uint8_t spi2_test_tx_dma[SPI2_TEST_BUFFER_BYTES]{};
alignas(64) uint8_t spi2_test_tx[SPI2_TEST_BUFFER_BYTES]{};
alignas(64) uint8_t spi2_test_rx[SPI2_TEST_BUFFER_BYTES]{};
LibXR::Thread spi2_test_thread;

void PublishSpi2TestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (SPI2_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = SPI2_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void ConfigureSpi2TestPinmux()
{
  constexpr uintptr_t PINMUX_BASE = 0x03001000u;
  LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_D3) =
      (LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_D3) & ~0x7u) | 1u;
  LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_D0) =
      (LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_D0) & ~0x7u) | 1u;
  LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_CMD) =
      (LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_CMD) & ~0x7u) | 1u;
  LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_CLK) =
      (LibXR::Register32(PINMUX_BASE, SPI2_TEST_PINMUX_SD1_CLK) & ~0x7u) | 1u;
}

#ifdef SG200X_SPI2_TEST
volatile bool spi2_test_completed = false;

void Spi2TestComplete(bool, void*, LibXR::ErrorCode result)
{
  spi2_test_completed = true;
  if (result != LibXR::ErrorCode::OK)
  {
    PublishSpi2TestResult(2u, static_cast<uint32_t>(-static_cast<int8_t>(result)));
    return;
  }
  uint32_t mismatch = SPI2_TEST_BYTES;
  for (uint32_t index = 0u; index < SPI2_TEST_BYTES; ++index)
  {
    if (spi2_test_rx[index] != spi2_test_tx[index])
    {
      mismatch = index;
      break;
    }
  }
  PublishSpi2TestResult(mismatch == SPI2_TEST_BYTES ? 0u : 1u, mismatch);
}

void StartSpi2Test(void*)
{
  PublishSpi2TestResult(0xFFu, 0u);
  ConfigureSpi2TestPinmux();
  for (uint32_t index = 0u; index < SPI2_TEST_BYTES; ++index)
  {
    spi2_test_tx[index] = static_cast<uint8_t>((index * 73u) ^ 0x5Au);
    spi2_test_rx[index] = 0u;
  }

  static LibXR::SG200XSPI spi(
      LibXR::SG200XRCC::SpiController::SPI2, 0u,
      {spi2_test_rx_dma, sizeof(spi2_test_rx_dma)},
      {spi2_test_tx_dma, sizeof(spi2_test_tx_dma)},
      {.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_1,
       .prescaler = LibXR::SPI::Prescaler::DIV_256});
  if (!spi.IsValid())
  {
    PublishSpi2TestResult(3u, 0u);
    return;
  }
  PublishSpi2TestResult(0xFEu, spi.ActualBusSpeed());

  static auto callback =
      LibXR::Callback<LibXR::ErrorCode>::Create<void*>(Spi2TestComplete, nullptr);
  LibXR::WriteOperation operation(callback);
  PublishSpi2TestResult(0xFDu, 0u);
  spi2_test_completed = false;
  const LibXR::ErrorCode transfer =
      spi.ReadAndWrite({spi2_test_rx, SPI2_TEST_BYTES},
                       {spi2_test_tx, SPI2_TEST_BYTES}, operation);
  if (transfer != LibXR::ErrorCode::OK)
  {
    PublishSpi2TestResult(2u, static_cast<uint32_t>(-static_cast<int8_t>(transfer)));
    return;
  }
  // Wait for the DMA completion callback.  This deliberately exercises the
  // same interrupt-driven completion path used by normal applications; a
  // successful buffer transfer without this callback is not a valid result.
  const uint64_t wait_until = LibXR::Timebase::GetMicroseconds() + 100000u;
  while (LibXR::Timebase::GetMicroseconds() < wait_until)
  {
    if (spi2_test_completed)
    {
      break;
    }
    asm volatile("nop");
  }
  if (!spi2_test_completed)
  {
    PublishSpi2TestResult(4u, 0u);  // DMA IRQ/callback timeout.
  }
}
#endif

#ifdef SG200X_SPI2_STRESS_TEST
#ifndef SG200X_SPI2_STRESS_ITERATIONS
#define SG200X_SPI2_STRESS_ITERATIONS 4096u
#endif

constexpr uint32_t SPI2_STRESS_ITERATIONS = SG200X_SPI2_STRESS_ITERATIONS;
constexpr uint32_t SPI2_STRESS_TIMEOUT_US = 250000u;
constexpr uint32_t SPI2_STRESS_BLOCK_TIMEOUT_TICKS = 1000u;
constexpr uint32_t SPI2_STRESS_COMPLETION_PENDING = 0u;
constexpr uint32_t SPI2_STRESS_COMPLETION_BIAS = 128u;
constexpr uint32_t SPI2_STRESS_STAGE_CONFIG = 1u;
constexpr uint32_t SPI2_STRESS_STAGE_RW = 2u;
constexpr uint32_t SPI2_STRESS_STAGE_TRANSFER = 3u;
constexpr uint32_t SPI2_STRESS_STAGE_WRITE = 4u;
constexpr uint32_t SPI2_STRESS_STAGE_READ = 5u;
constexpr uint32_t SPI2_STRESS_STAGE_MEM_WRITE = 6u;
constexpr uint32_t SPI2_STRESS_STAGE_MEM_READ = 7u;
constexpr uint32_t SPI2_STRESS_STAGE_BUSY = 8u;
constexpr uint32_t SPI2_STRESS_STAGE_BLOCK = 9u;
constexpr uint32_t SPI2_STRESS_STAGE_ARGUMENTS = 10u;
constexpr uint32_t SPI2_STRESS_STAGE_LOOP = 11u;
constexpr uint32_t SPI2_STRESS_STAGE_CIRCULAR = 12u;
constexpr uint32_t SPI2_STRESS_CIRCULAR_BYTES = 64u;
constexpr uint32_t SPI2_STRESS_CIRCULAR_TIMEOUT_US = 30000000u;

// The completion path crosses the DMA ISR/task boundary. Volatile is not a
// synchronization primitive for that handoff: it does not establish ordering
// for the result or for DMA-invalidated receive data.
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "C906L stress-test handoff must not use a library lock in an ISR");
std::atomic<uint32_t> spi2_stress_completion{0u};

struct Spi2CircularStressState
{
  LibXR::SG200XSPI* spi = nullptr;
  std::atomic<uint32_t> cycles{0u};
  std::atomic<uint32_t> status{0u};
  std::atomic<uint32_t> detail{0u};
};

Spi2CircularStressState spi2_circular_stress{};

uint32_t FindSpi2StressMismatch(const uint8_t* expected, const uint8_t* actual,
                                uint32_t size);

void Spi2StressComplete(bool, void*, LibXR::ErrorCode result)
{
  spi2_stress_completion.store(
      static_cast<uint32_t>(static_cast<int8_t>(result) + SPI2_STRESS_COMPLETION_BIAS),
      std::memory_order_release);
}

void Spi2CircularStressComplete(bool in_isr, Spi2CircularStressState* state,
                                LibXR::ErrorCode result)
{
  if (result != LibXR::ErrorCode::OK)
  {
    state->detail.store(static_cast<uint32_t>(-static_cast<int8_t>(result)),
                        std::memory_order_relaxed);
    state->status.store(2u, std::memory_order_release);
    return;
  }

  const uint32_t mismatch =
      FindSpi2StressMismatch(spi2_test_tx_dma, spi2_test_rx_dma,
                             SPI2_STRESS_CIRCULAR_BYTES);
  if (mismatch != SPI2_STRESS_CIRCULAR_BYTES)
  {
    const LibXR::ErrorCode stop_result = state->spi->StopCircularTransfer(in_isr);
    state->detail.store(stop_result == LibXR::ErrorCode::OK
                            ? mismatch
                            : static_cast<uint32_t>(-static_cast<int8_t>(stop_result)),
                        std::memory_order_relaxed);
    state->status.store(3u, std::memory_order_release);
    return;
  }

  const uint32_t cycles = state->cycles.fetch_add(1u, std::memory_order_relaxed) + 1u;
  if (cycles == SPI2_STRESS_ITERATIONS)
  {
    const LibXR::ErrorCode stop_result = state->spi->StopCircularTransfer(in_isr);
    state->detail.store(stop_result == LibXR::ErrorCode::OK
                            ? cycles
                            : static_cast<uint32_t>(-static_cast<int8_t>(stop_result)),
                        std::memory_order_relaxed);
    state->status.store(stop_result == LibXR::ErrorCode::OK ? 1u : 4u,
                        std::memory_order_release);
  }
}

uint32_t Spi2StressDetail(uint32_t stage, uint32_t iteration, uint32_t offset)
{
  return ((stage & 0x0Fu) << 20u) | ((iteration & 0x03FFu) << 10u) |
         (offset & 0x03FFu);
}

bool Spi2StressFail(uint32_t result, uint32_t stage, uint32_t iteration,
                    uint32_t offset = 0u)
{
  PublishSpi2TestResult(result, Spi2StressDetail(stage, iteration, offset));
  return false;
}

void FillSpi2StressPattern(uint8_t* data, uint32_t size, uint32_t salt)
{
  for (uint32_t index = 0u; index < size; ++index)
  {
    data[index] = static_cast<uint8_t>((index * 73u) ^ (salt * 29u) ^
                                       (salt >> ((index & 3u) * 8u)) ^ 0xA5u);
  }
}

uint32_t FindSpi2StressMismatch(const uint8_t* expected, const uint8_t* actual,
                                 uint32_t size)
{
  for (uint32_t index = 0u; index < size; ++index)
  {
    if (expected[index] != actual[index])
    {
      return index;
    }
  }
  return size;
}

bool WaitForSpi2StressCompletion(LibXR::ErrorCode start_result, uint32_t stage,
                                 uint32_t iteration)
{
  if (start_result != LibXR::ErrorCode::OK)
  {
    return Spi2StressFail(2u, stage, iteration,
                          static_cast<uint32_t>(-static_cast<int8_t>(start_result)));
  }

  PublishSpi2TestResult(0xF9u, (stage << 12u) | iteration);
  const uint64_t wait_until = LibXR::Timebase::GetMicroseconds() + SPI2_STRESS_TIMEOUT_US;
  PublishSpi2TestResult(0xF8u, (stage << 12u) | iteration);
  while (spi2_stress_completion.load(std::memory_order_acquire) ==
             SPI2_STRESS_COMPLETION_PENDING &&
          LibXR::Timebase::GetMicroseconds() < wait_until)
  {
  }

  const uint32_t completion = spi2_stress_completion.load(std::memory_order_acquire);
  if (completion == SPI2_STRESS_COMPLETION_PENDING)
  {
    return Spi2StressFail(4u, stage, iteration);
  }
  const auto completion_result = static_cast<LibXR::ErrorCode>(
      static_cast<int8_t>(completion - SPI2_STRESS_COMPLETION_BIAS));
  if (completion_result != LibXR::ErrorCode::OK)
  {
    return Spi2StressFail(
        2u, stage, iteration,
        static_cast<uint32_t>(-static_cast<int8_t>(completion_result)));
  }
  return true;
}

bool VerifySpi2StressLoopback(const uint8_t* expected, const uint8_t* actual, uint32_t size,
                              uint32_t stage, uint32_t iteration)
{
  const uint32_t mismatch = FindSpi2StressMismatch(expected, actual, size);
  return mismatch == size || Spi2StressFail(1u, stage, iteration, mismatch);
}

void StartSpi2StressTest(void*)
{
  PublishSpi2TestResult(0xFFu, 0u);
  ConfigureSpi2TestPinmux();
  static LibXR::SG200XSPI spi(
      LibXR::SG200XRCC::SpiController::SPI2, 0u,
      {spi2_test_rx_dma, sizeof(spi2_test_rx_dma)},
      {spi2_test_tx_dma, sizeof(spi2_test_tx_dma)},
      {.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_1,
       .prescaler = LibXR::SPI::Prescaler::DIV_256});
  if (!spi.IsValid())
  {
    PublishSpi2TestResult(3u, 0u);
    return;
  }
  PublishSpi2TestResult(0xFEu, spi.ActualBusSpeed());

  static auto callback =
      LibXR::Callback<LibXR::ErrorCode>::Create<void*>(Spi2StressComplete, nullptr);
  LibXR::WriteOperation operation(callback);
  static auto circular_callback =
      LibXR::Callback<LibXR::ErrorCode>::Create<Spi2CircularStressState*>(
          Spi2CircularStressComplete, &spi2_circular_stress);
  LibXR::WriteOperation circular_operation(circular_callback);
  static LibXR::Semaphore block_semaphore{};
  const LibXR::SPI::Configuration configurations[] = {
      {.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_1,
       .prescaler = LibXR::SPI::Prescaler::DIV_256},
      {.clock_polarity = LibXR::SPI::ClockPolarity::HIGH,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_1,
       .prescaler = LibXR::SPI::Prescaler::DIV_128},
      {.clock_polarity = LibXR::SPI::ClockPolarity::LOW,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_2,
       .prescaler = LibXR::SPI::Prescaler::DIV_256},
      {.clock_polarity = LibXR::SPI::ClockPolarity::HIGH,
       .clock_phase = LibXR::SPI::ClockPhase::EDGE_2,
       .prescaler = LibXR::SPI::Prescaler::DIV_128},
  };
  constexpr uint32_t functional_sizes[] = {1u, 3u, 8u, 17u, 64u, 127u,
                                             SPI2_TEST_BUFFER_BYTES};

  if (spi.GetMaxBusSpeed() !=
          LibXR::SG200XRCC::Instance().ClockRate(LibXR::SG200XRCC::ClockId::Spi) / 2u ||
      spi.GetMaxPrescaler() != LibXR::SPI::Prescaler::DIV_16384)
  {
    Spi2StressFail(5u, SPI2_STRESS_STAGE_CONFIG, 0u);
    return;
  }

  PublishSpi2TestResult(0xFDu, 0u);

  // Exercise every CPOL/CPHA mode and the short-to-full-buffer DMA boundary.
  // A MOSI-to-MISO loopback verifies both directions without assuming a
  // particular external peripheral register map.
  for (uint32_t config_index = 0u;
       config_index < sizeof(configurations) / sizeof(configurations[0]); ++config_index)
  {
    PublishSpi2TestResult(0xFCu, config_index);
    if (spi.SetConfig(configurations[config_index]) != LibXR::ErrorCode::OK ||
        spi.GetBusSpeed() != spi.ActualBusSpeed())
    {
      Spi2StressFail(5u, SPI2_STRESS_STAGE_CONFIG, config_index);
      return;
    }

    for (uint32_t size_index = 0u;
         size_index < sizeof(functional_sizes) / sizeof(functional_sizes[0]); ++size_index)
    {
      const uint32_t size = functional_sizes[size_index];
      FillSpi2StressPattern(spi2_test_tx, size, (config_index << 8u) | size_index);
      for (uint32_t index = 0u; index < size; ++index)
      {
        spi2_test_rx[index] = 0u;
      }
      spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING,
                                   std::memory_order_relaxed);
      PublishSpi2TestResult(0xFBu, (config_index << 8u) | size_index);
      const LibXR::ErrorCode start_result =
          spi.ReadAndWrite({spi2_test_rx, size}, {spi2_test_tx, size}, operation);
      PublishSpi2TestResult(0xFAu,
                            static_cast<uint32_t>(-static_cast<int8_t>(start_result)));
      if (!WaitForSpi2StressCompletion(start_result, SPI2_STRESS_STAGE_RW,
                                       (config_index << 8u) | size_index) ||
          !VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx, size,
                                    SPI2_STRESS_STAGE_RW,
                                    (config_index << 8u) | size_index))
      {
        return;
      }
    }
  }

  // STM32SPI stages ordinary calls through its configured DMA buffers, so
  // caller buffers do not inherit DMA cache-line alignment requirements.
  constexpr uint32_t unaligned_size = 17u;
  uint8_t* const unaligned_tx = spi2_test_tx + 1u;
  uint8_t* const unaligned_rx = spi2_test_rx + 3u;
  FillSpi2StressPattern(unaligned_tx, unaligned_size, 0x10203u);
  for (uint32_t index = 0u; index < unaligned_size; ++index)
  {
    unaligned_rx[index] = 0u;
  }
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING,
                               std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(
          spi.ReadAndWrite({unaligned_rx, unaligned_size},
                           {unaligned_tx, unaligned_size}, operation),
          SPI2_STRESS_STAGE_RW, 0x3FFu) ||
      !VerifySpi2StressLoopback(unaligned_tx, unaligned_rx, unaligned_size,
                                SPI2_STRESS_STAGE_RW, 0x3FFu))
  {
    return;
  }

  if (spi.SetConfig(configurations[0]) != LibXR::ErrorCode::OK)
  {
    Spi2StressFail(5u, SPI2_STRESS_STAGE_CONFIG, 0u);
    return;
  }

  // Keep both DMAC channels on self-linked LLIs, validate every completed RX
  // block, then stop from the final ISR callback. A subsequent Normal transfer
  // below proves that abort/release left the SSI and channel pool reusable.
  FillSpi2StressPattern(spi2_test_tx_dma, SPI2_STRESS_CIRCULAR_BYTES, 0xC1ACu);
  for (uint32_t index = 0u; index < SPI2_STRESS_CIRCULAR_BYTES; ++index)
  {
    spi2_test_rx_dma[index] = 0u;
  }
  spi2_circular_stress.spi = &spi;
  spi2_circular_stress.cycles.store(0u, std::memory_order_relaxed);
  spi2_circular_stress.status.store(0u, std::memory_order_relaxed);
  spi2_circular_stress.detail.store(0u, std::memory_order_relaxed);
  LibXR::WriteOperation circular_block_operation(block_semaphore,
                                                 SPI2_STRESS_BLOCK_TIMEOUT_TICKS);
  if (spi.StartCircularTransfer(SPI2_STRESS_CIRCULAR_BYTES,
                                circular_block_operation) !=
          LibXR::ErrorCode::NOT_SUPPORT ||
      spi.StartCircularTransfer(0u, circular_operation) != LibXR::ErrorCode::SIZE_ERR ||
      spi.StartCircularTransfer(SPI2_STRESS_CIRCULAR_BYTES, circular_operation, true) !=
          LibXR::ErrorCode::NOT_SUPPORT)
  {
    Spi2StressFail(5u, SPI2_STRESS_STAGE_CIRCULAR, 0u);
    return;
  }
  const LibXR::ErrorCode circular_start =
      spi.StartCircularTransfer(SPI2_STRESS_CIRCULAR_BYTES, circular_operation);
  if (circular_start != LibXR::ErrorCode::OK || !spi.IsCircularTransferActive() ||
      spi.SetConfig(configurations[0]) != LibXR::ErrorCode::BUSY ||
      spi.Transfer(1u, operation) != LibXR::ErrorCode::BUSY ||
      spi.StartCircularTransfer(SPI2_STRESS_CIRCULAR_BYTES, circular_operation) !=
          LibXR::ErrorCode::BUSY)
  {
    if (spi.IsCircularTransferActive())
    {
      (void)spi.StopCircularTransfer();
    }
    Spi2StressFail(5u, SPI2_STRESS_STAGE_CIRCULAR, 1u);
    return;
  }
  const uint64_t circular_wait_until =
      LibXR::Timebase::GetMicroseconds() + SPI2_STRESS_CIRCULAR_TIMEOUT_US;
  while (spi2_circular_stress.status.load(std::memory_order_acquire) == 0u &&
         LibXR::Timebase::GetMicroseconds() < circular_wait_until)
  {
  }
  const uint32_t circular_status =
      spi2_circular_stress.status.load(std::memory_order_acquire);
  if (circular_status == 0u)
  {
    (void)spi.StopCircularTransfer();
    Spi2StressFail(4u, SPI2_STRESS_STAGE_CIRCULAR,
                   spi2_circular_stress.cycles.load(std::memory_order_acquire));
    return;
  }
  if (circular_status != 1u || spi.IsCircularTransferActive() ||
      spi2_circular_stress.cycles.load(std::memory_order_acquire) !=
          SPI2_STRESS_ITERATIONS ||
      spi.StopCircularTransfer() != LibXR::ErrorCode::STATE_ERR)
  {
    Spi2StressFail(circular_status == 3u ? 1u : 2u,
                   SPI2_STRESS_STAGE_CIRCULAR,
                   spi2_circular_stress.cycles.load(std::memory_order_acquire),
                   spi2_circular_stress.detail.load(std::memory_order_acquire));
    return;
  }

  // Transfer uses the active DMA buffers directly. It is the zero-copy API
  // used by STM32SPI clients, so validate it independently of ReadAndWrite.
  FillSpi2StressPattern(spi2_test_tx_dma, 127u, 0x1234u);
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.Transfer(127u, operation),
                                   SPI2_STRESS_STAGE_TRANSFER, 0u) ||
      !VerifySpi2StressLoopback(spi2_test_tx_dma, spi2_test_rx_dma, 127u,
                                SPI2_STRESS_STAGE_TRANSFER, 0u))
  {
    return;
  }

  // Cover one-direction helpers. The driver still clocks both SSI directions;
  // the loopback lets this test inspect the otherwise discarded RX staging data.
  FillSpi2StressPattern(spi2_test_tx, 31u, 0x2345u);
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.Write({spi2_test_tx, 31u}, operation),
                                   SPI2_STRESS_STAGE_WRITE, 0u) ||
      !VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx_dma, 31u,
                                SPI2_STRESS_STAGE_WRITE, 0u))
  {
    return;
  }

  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.Read({spi2_test_rx, 31u}, operation),
                                   SPI2_STRESS_STAGE_READ, 0u))
  {
    return;
  }
  for (uint32_t index = 0u; index < 31u; ++index)
  {
    if (spi2_test_rx[index] != 0xFFu)
    {
      Spi2StressFail(1u, SPI2_STRESS_STAGE_READ, 0u, index);
      return;
    }
  }

  FillSpi2StressPattern(spi2_test_tx, 31u, 0x3456u);
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.MemWrite(0x2Au, {spi2_test_tx, 31u}, operation),
                                   SPI2_STRESS_STAGE_MEM_WRITE, 0u))
  {
    return;
  }
  if (spi2_test_rx_dma[0] != 0x2Au)
  {
    Spi2StressFail(1u, SPI2_STRESS_STAGE_MEM_WRITE, 0u, 0u);
    return;
  }
  if (!VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx_dma + 1u, 31u,
                                SPI2_STRESS_STAGE_MEM_WRITE, 0u))
  {
    return;
  }

  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.MemRead(0x2Au, {spi2_test_rx, 31u}, operation),
                                   SPI2_STRESS_STAGE_MEM_READ, 0u))
  {
    return;
  }
  if (spi2_test_rx_dma[0] != 0xAAu)
  {
    Spi2StressFail(1u, SPI2_STRESS_STAGE_MEM_READ, 0u, 0u);
    return;
  }
  for (uint32_t index = 0u; index < 31u; ++index)
  {
    if (spi2_test_rx_dma[index + 1u] != 0xFFu || spi2_test_rx[index] != 0xFFu)
    {
      Spi2StressFail(1u, SPI2_STRESS_STAGE_MEM_READ, 0u, index + 1u);
      return;
    }
  }

  // A normal client may issue a BLOCK operation instead of a callback. This
  // validates the same ISR-to-task handoff and leaves the instance reusable.
  FillSpi2StressPattern(spi2_test_tx, 127u, 0x4567u);
  for (uint32_t index = 0u; index < 127u; ++index)
  {
    spi2_test_rx[index] = 0u;
  }
  LibXR::WriteOperation block_operation(block_semaphore, SPI2_STRESS_BLOCK_TIMEOUT_TICKS);
  const LibXR::ErrorCode block_result =
      spi.ReadAndWrite({spi2_test_rx, 127u}, {spi2_test_tx, 127u}, block_operation);
  if (block_result != LibXR::ErrorCode::OK)
  {
    Spi2StressFail(2u, SPI2_STRESS_STAGE_BLOCK, 0u,
                   static_cast<uint32_t>(-static_cast<int8_t>(block_result)));
    return;
  }
  if (!VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx, 127u,
                                SPI2_STRESS_STAGE_BLOCK, 0u))
  {
    return;
  }

  // Reject bad inputs without invoking a callback, as the STM32-facing SPI
  // contract requires callers to receive the immediate argument error.
  if (spi.ReadAndWrite({nullptr, 1u}, {spi2_test_tx, 1u}, operation) !=
          LibXR::ErrorCode::ARG_ERR ||
      spi.ReadAndWrite({}, {}, operation, true) != LibXR::ErrorCode::NOT_SUPPORT ||
      spi.MemWrite(0x2Au, {nullptr, 1u}, operation) != LibXR::ErrorCode::ARG_ERR ||
      spi.MemRead(0x2Au, {nullptr, 1u}, operation) != LibXR::ErrorCode::ARG_ERR ||
      spi.MemWrite(0x100u, {spi2_test_tx, 1u}, operation) != LibXR::ErrorCode::SIZE_ERR ||
      spi.MemRead(0x100u, {spi2_test_rx, 1u}, operation) != LibXR::ErrorCode::SIZE_ERR)
  {
    Spi2StressFail(5u, SPI2_STRESS_STAGE_ARGUMENTS, 0u);
    return;
  }
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.MemWrite(0x2Au, {}, operation),
                                   SPI2_STRESS_STAGE_ARGUMENTS, 1u))
  {
    return;
  }
  if (spi2_test_rx_dma[0] != 0x2Au)
  {
    Spi2StressFail(1u, SPI2_STRESS_STAGE_ARGUMENTS, 1u,
                   spi2_test_rx_dma[0]);
    return;
  }
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  if (!WaitForSpi2StressCompletion(spi.MemRead(0x2Au, {}, operation),
                                   SPI2_STRESS_STAGE_ARGUMENTS, 2u))
  {
    return;
  }
  if (spi2_test_rx_dma[0] != 0xAAu)
  {
    Spi2StressFail(1u, SPI2_STRESS_STAGE_ARGUMENTS, 2u,
                   spi2_test_rx_dma[0]);
    return;
  }

  // Start a long transfer, then verify that both a reconfiguration and a
  // second transfer observe BUSY until DMA completion releases the instance.
  FillSpi2StressPattern(spi2_test_tx, SPI2_TEST_BUFFER_BYTES, 0x5678u);
  spi2_stress_completion.store(SPI2_STRESS_COMPLETION_PENDING, std::memory_order_relaxed);
  const LibXR::ErrorCode busy_start =
      spi.ReadAndWrite({spi2_test_rx, SPI2_TEST_BUFFER_BYTES},
                       {spi2_test_tx, SPI2_TEST_BUFFER_BYTES}, operation);
  if (busy_start != LibXR::ErrorCode::OK)
  {
    Spi2StressFail(2u, SPI2_STRESS_STAGE_BUSY, 0u,
                   static_cast<uint32_t>(-static_cast<int8_t>(busy_start)));
    return;
  }
  if (spi.SetConfig(configurations[0]) != LibXR::ErrorCode::BUSY ||
      spi.ReadAndWrite({spi2_test_rx, 1u}, {spi2_test_tx, 1u}, operation) !=
          LibXR::ErrorCode::BUSY)
  {
    Spi2StressFail(5u, SPI2_STRESS_STAGE_BUSY, 0u);
    return;
  }
  if (!WaitForSpi2StressCompletion(LibXR::ErrorCode::OK, SPI2_STRESS_STAGE_BUSY, 0u) ||
      !VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx, SPI2_TEST_BUFFER_BYTES,
                                SPI2_STRESS_STAGE_BUSY, 0u))
  {
    return;
  }

  // Finally sustain back-to-back DMA transfers with varying lengths and fresh
  // payloads. This catches descriptor/channel release leaks that a single
  // smoke transfer cannot expose.
  for (uint32_t iteration = 0u; iteration < SPI2_STRESS_ITERATIONS; ++iteration)
  {
    const uint32_t size = functional_sizes[iteration %
                                           (sizeof(functional_sizes) /
                                            sizeof(functional_sizes[0]))];
    FillSpi2StressPattern(spi2_test_tx, size, iteration + 0x6000u);
    for (uint32_t index = 0u; index < size; ++index)
    {
      spi2_test_rx[index] = 0u;
    }
    spi2_stress_completion.store(0u, std::memory_order_relaxed);
    if (!WaitForSpi2StressCompletion(
        spi.ReadAndWrite({spi2_test_rx, size}, {spi2_test_tx, size}, operation),
              SPI2_STRESS_STAGE_LOOP, iteration) ||
        !VerifySpi2StressLoopback(spi2_test_tx, spi2_test_rx, size,
                                  SPI2_STRESS_STAGE_LOOP, iteration))
    {
      return;
    }
  }
  PublishSpi2TestResult(0u, SPI2_STRESS_ITERATIONS);
}

#endif
#endif

#ifdef SG200X_ADC_TEST
constexpr uint32_t ADC_TEST_TRACE_MAGIC = 0xADC90601u;
constexpr uint32_t ADC_TEST_CHANNELS = 6u;

void StartAdcTest(void*)
{
  // Keep the object alive for repeated timer callbacks. The trace layout is:
  // words 16..17 magic/sequence, word 18 controller status, then six pairs
  // of raw/error and millivolts for logical channels 1..6.
  static LibXR::SG200XADC adc;
  static uint32_t sequence = 0u;
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[16] = ADC_TEST_TRACE_MAGIC;
  trace[17] = ++sequence;
  trace[18] = adc.IsValid() ? 0u : 1u;
  for (uint32_t index = 0u; index < ADC_TEST_CHANNELS; ++index)
  {
    uint16_t raw = 0u;
    const LibXR::ErrorCode result = adc.ReadRaw(static_cast<uint8_t>(index), raw);
    const uint32_t code = result == LibXR::ErrorCode::OK
                              ? 0u
                              : static_cast<uint32_t>(-static_cast<int8_t>(result));
    trace[19u + index * 2u] = static_cast<uint32_t>(raw) | (code << 16u);
    trace[20u + index * 2u] =
        result == LibXR::ErrorCode::OK ? static_cast<uint32_t>(raw) * 1800u / 4095u : 0u;
  }
  flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 64u);
}
#endif

#ifdef SG200X_PWM_ADC_TEST
constexpr uint32_t PWM_ADC_TEST_MAGIC = 0x50414443u;  // "PADC"
constexpr uint32_t PWM_ADC_TEST_DUTY_COUNT = 5u;
constexpr uint32_t PWM_ADC_TEST_SAMPLES = 32u;
constexpr uint32_t PWM_ADC_TEST_TRACE_BASE = 21u;
constexpr uintptr_t PWM_ADC_TEST_PINMUX_BASE = 0x03001000u;
constexpr uint32_t PWM_ADC_TEST_PINMUX_A18 = 0x68u;
constexpr uint32_t PWM_ADC_TEST_PINMUX_ADC1 = 0xF8u;

struct PwmAdcDutyResult
{
  uint32_t duty_permille;
  uint32_t min_raw;
  uint32_t max_raw;
  uint32_t average_raw;
  uint32_t average_mv;
  uint32_t successful_samples;
  uint32_t first_error;
};

static_assert(sizeof(PwmAdcDutyResult) == 7u * sizeof(uint32_t));

void ConfigurePwmAdcPinmux()
{
  // Use the exposed PWM6 next to the ADC header. GPIOP20 is also labelled PWM6
  // but is wired to the Wi-Fi SDIO1_D1 signal and must remain untouched.
  auto& a18 = LibXR::Register32(PWM_ADC_TEST_PINMUX_BASE, PWM_ADC_TEST_PINMUX_A18);
  a18 = (a18 & ~0x7u) | 2u;  // JTAG_CPU_TCK -> PWM_6
  auto& adc1 = LibXR::Register32(PWM_ADC_TEST_PINMUX_BASE, PWM_ADC_TEST_PINMUX_ADC1);
  adc1 = (adc1 & ~0x7u) | 3u;  // ADC1 -> XGPIOB_3
}

void StartPwmAdcTest(void*)
{
  static LibXR::SG200XPWM pwm(
      LibXR::SG200XPWM::Channel::PWM6, 100000000u,
      {PWM_ADC_TEST_PINMUX_A18, 2u});
  static LibXR::SG200XADC adc({1u}, LibXR::SG200XADC::Config{});
  static constexpr uint16_t duties[PWM_ADC_TEST_DUTY_COUNT] = {0u, 250u, 500u, 750u,
                                                                1000u};
  static PwmAdcDutyResult results[PWM_ADC_TEST_DUTY_COUNT]{};
  static uint32_t sequence = 0u;
  static uint32_t duty_index = 0u;
  static uint32_t status = 0u;
  static bool configured = false;

  ConfigurePwmAdcPinmux();
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[16] = PWM_ADC_TEST_MAGIC;
  trace[17] = ++sequence;
  trace[18] = status;
  trace[19] = pwm.IsValid() ? 0u : 1u;
  trace[20] = adc.IsValid() ? 0u : 1u;

  if (!pwm.IsValid() || !adc.IsValid())
  {
    trace[18] = 1u;
    flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 20u);
    return;
  }

  if (!configured)
  {
    status |= static_cast<uint32_t>(pwm.SetPolarity(true));
    status |= static_cast<uint32_t>(pwm.SetConfig({100u})) << 8u;
    status |= static_cast<uint32_t>(pwm.Enable()) << 16u;
    status |= static_cast<uint32_t>(pwm.SetDutyCycle(0.0f)) << 24u;
    configured = true;
    trace[18] = status;
    flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 20u);
    return;
  }

  // One callback period is 250 ms, so the PWM output has been stable for many
  // cycles before sampling. ADC1 is the first entry in the {1} channel list.
  uint32_t min_raw = LibXR::SG200XADC::MAX_RAW;
  uint32_t max_raw = 0u;
  uint64_t sum_raw = 0u;
  uint32_t successful = 0u;
  uint32_t first_error = results[duty_index].first_error;
  for (uint32_t sample = 0u; sample < PWM_ADC_TEST_SAMPLES; ++sample)
  {
    uint16_t raw = 0u;
    const auto result = adc.ReadRaw(0u, raw);
    if (result != LibXR::ErrorCode::OK)
    {
      if (first_error == 0u)
      {
        first_error = static_cast<uint32_t>(-static_cast<int8_t>(result));
      }
      continue;
    }
    min_raw = raw < min_raw ? raw : min_raw;
    max_raw = raw > max_raw ? raw : max_raw;
    sum_raw += raw;
    ++successful;
    // Spread samples across multiple PWM periods instead of repeatedly
    // sampling the same phase of a fast output waveform.
    LibXR::Thread::Sleep(1u);
  }
  results[duty_index] = {
      duties[duty_index],
      successful == 0u ? 0u : min_raw,
      successful == 0u ? 0u : max_raw,
      successful == 0u ? 0u : static_cast<uint32_t>(sum_raw / successful),
      successful == 0u
          ? 0u
          : static_cast<uint32_t>((sum_raw / successful) * 1800u /
                                   LibXR::SG200XADC::MAX_RAW),
      successful,
      first_error,
  };

  volatile uint32_t* const result_words = trace + PWM_ADC_TEST_TRACE_BASE;
  const volatile uint32_t* const source =
      reinterpret_cast<const volatile uint32_t*>(&results[0]);
  for (uint32_t index = 0u; index < PWM_ADC_TEST_DUTY_COUNT * 7u; ++index)
  {
    result_words[index] = source[index];
  }
  trace[18] |= (successful == PWM_ADC_TEST_SAMPLES ? 0u : 1u) << 24u;
  status = trace[18];
  flush_dcache_range(BOOT_TRACE_ADDRESS + 64u,
                     (PWM_ADC_TEST_TRACE_BASE + PWM_ADC_TEST_DUTY_COUNT * 7u) *
                         sizeof(uint32_t) - 64u);

  duty_index = (duty_index + 1u) % PWM_ADC_TEST_DUTY_COUNT;
  const auto duty_result = pwm.SetDutyCycle(
      static_cast<float>(duties[duty_index]) / 1000.0f);
  if (duty_result != LibXR::ErrorCode::OK)
  {
    results[duty_index].first_error =
        static_cast<uint32_t>(-static_cast<int8_t>(duty_result));
  }
}
#endif

#ifdef SG200X_RCC_TEST
constexpr uint32_t RCC_TEST_MAGIC = 0x52434331u;  // "RCC1"
constexpr uint32_t RCC_TEST_TRACE_EVENT = 1u;
constexpr uint32_t RCC_TEST_WORKERS = 2u;
constexpr uint32_t RCC_TEST_ITERATIONS = 10000u;
constexpr uint32_t RCC_TEST_MAX_CONTENTION_RETRIES = 64u;
constexpr uint32_t RCC_TEST_DIAGNOSTIC_WORD = 40u;
constexpr uintptr_t RCC_TEST_CLOCK_BASE = 0x03002000u;
constexpr uintptr_t RCC_TEST_RESET_BASE = 0x03003000u;

alignas(64) std::atomic<uint32_t> rcc_test_errors{0u};
alignas(64) std::atomic<uint32_t> rcc_test_done{0u};
alignas(64) std::atomic<uint32_t> rcc_test_iterations{0u};
alignas(64) std::atomic<uint32_t> rcc_test_contentions{0u};
alignas(64) std::atomic<bool> rcc_test_low_started{false};
alignas(64) std::atomic<uint32_t> rcc_test_preempt_iteration{UINT32_MAX};
LibXR::Thread rcc_test_workers[RCC_TEST_WORKERS];

void PublishRccTestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (RCC_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = RCC_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void PublishRccTestDiagnostics(uint32_t errors)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  auto& rcc = LibXR::SG200XRCC::Instance();
  volatile uint32_t* const diagnostics = trace + RCC_TEST_DIAGNOSTIC_WORD;
  diagnostics[0] = RCC_TEST_MAGIC;
  diagnostics[1] = errors;
  diagnostics[2] = rcc_test_iterations.load(std::memory_order_acquire);
  diagnostics[3] = rcc.ClockRate(LibXR::SG200XRCC::ClockId::Fpll);
  diagnostics[4] = rcc.ClockRate(LibXR::SG200XRCC::ClockId::Axi4);
  diagnostics[5] = rcc.ClockRate(LibXR::SG200XRCC::ClockId::Axi6);
  diagnostics[6] = rcc.ClockRate(LibXR::SG200XRCC::ClockId::Spi);
  diagnostics[7] = rcc.ClockRate(LibXR::SG200XRCC::ClockId::I2c);
  diagnostics[8] = LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x100u);
  diagnostics[9] = LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x104u);
  diagnostics[10] = LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x00Cu);
  diagnostics[11] = LibXR::Register32(RCC_TEST_RESET_BASE);
  diagnostics[12] = LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x004u);
  diagnostics[13] = LibXR::Register32(RCC_TEST_RESET_BASE + 4u);
  diagnostics[14] = rcc_test_preempt_iteration.load(std::memory_order_acquire);
  diagnostics[15] = rcc_test_contentions.load(std::memory_order_acquire);
  flush_dcache_range(BOOT_TRACE_ADDRESS + RCC_TEST_DIAGNOSTIC_WORD * sizeof(uint32_t),
                     16u * sizeof(uint32_t));
}

[[nodiscard]] bool RccRegisterMatches(uintptr_t address, uint32_t mask,
                                      uint32_t expected) noexcept
{
  return (LibXR::Register32(address) & mask) == expected;
}

[[nodiscard]] uint32_t ValidateRccFinalState()
{
  uint32_t errors = 0u;
  const uint32_t divider_valid = LibXR::Bit(3u);
  if (!RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x0B8u,
                          (0xFu << 16u) | divider_valid,
                          (5u << 16u) | divider_valid) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x0BCu,
                          (0xFu << 16u) | divider_valid,
                          (15u << 16u) | divider_valid) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x0FCu,
                          (0x3Fu << 16u) | divider_valid,
                          (25u << 16u) | divider_valid) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x100u,
                          (0x3Fu << 16u) | divider_valid,
                          (8u << 16u) | divider_valid) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x104u,
                          (0xFu << 16u) | divider_valid,
                          (1u << 16u) | divider_valid))
  {
    errors |= 1u << 21u;
  }
  constexpr uint32_t CLOCK_GATE2_MASK =
      (1u << 5u) | (1u << 6u) | (1u << 7u) | (1u << 18u);
  if (!RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x008u,
                          (1u << 1u) | (1u << 2u),
                          (1u << 1u) | (1u << 2u)) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x00Cu, CLOCK_GATE2_MASK,
                          CLOCK_GATE2_MASK) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x004u, 1u << 11u,
                          1u << 11u) ||
      !RccRegisterMatches(RCC_TEST_RESET_BASE, 1u << 28u, 1u << 28u) ||
      !RccRegisterMatches(RCC_TEST_RESET_BASE + 4u, 1u << (42u - 32u),
                          1u << (42u - 32u)))
  {
    errors |= 1u << 22u;
  }
  return errors;
}

[[nodiscard]] LibXR::ErrorCode EnableRccClockWithRetry(
    LibXR::SG200XRCC::ClockId clock)
{
  auto& rcc = LibXR::SG200XRCC::Instance();
  for (uint32_t retry = 0u; retry < RCC_TEST_MAX_CONTENTION_RETRIES; ++retry)
  {
    const LibXR::ErrorCode result = rcc.EnableClock(clock);
    if (result != LibXR::ErrorCode::BUSY)
    {
      return result;
    }
    rcc_test_contentions.fetch_add(1u, std::memory_order_relaxed);
    // A higher-priority contender must block briefly so the preempted writer
    // can finish; yielding alone does not schedule a lower-priority task.
    LibXR::Thread::Sleep(1u);
  }
  return LibXR::ErrorCode::TIMEOUT;
}

[[nodiscard]] bool RccRateMatchesWithRetry(LibXR::SG200XRCC::ClockId clock,
                                           uint32_t expected)
{
  auto& rcc = LibXR::SG200XRCC::Instance();
  for (uint32_t retry = 0u; retry < RCC_TEST_MAX_CONTENTION_RETRIES; ++retry)
  {
    const uint32_t rate = rcc.ClockRate(clock);
    if (rate == expected)
    {
      return true;
    }
    if (rate != 0u)
    {
      return false;
    }
    rcc_test_contentions.fetch_add(1u, std::memory_order_relaxed);
    LibXR::Thread::Sleep(1u);
  }
  return false;
}

void RccTestWorker(uint32_t worker)
{
  auto& rcc = LibXR::SG200XRCC::Instance();
  uint32_t errors = 0u;
  if (worker == 0u)
  {
    rcc_test_low_started.store(true, std::memory_order_release);
  }
  else
  {
    // This high-priority worker is created first and blocks until the low
    // worker runs. Its tick-driven wakeup can preempt an in-flight RCC call.
    while (!rcc_test_low_started.load(std::memory_order_acquire))
    {
      LibXR::Thread::Sleep(1u);
    }
    const uint32_t preempt_iteration =
        rcc_test_iterations.load(std::memory_order_acquire);
    rcc_test_preempt_iteration.store(preempt_iteration, std::memory_order_release);
    if (preempt_iteration == 0u || preempt_iteration >= RCC_TEST_ITERATIONS)
    {
      errors |= 1u << 18u;
    }
  }
  for (uint32_t iteration = 0u; iteration < RCC_TEST_ITERATIONS; ++iteration)
  {
    const bool spi_worker = worker == 0u;
    const auto clock = spi_worker ? LibXR::SG200XRCC::ClockId::Spi
                                  : LibXR::SG200XRCC::ClockId::I2c;
    const auto bus_clock = spi_worker ? LibXR::SG200XRCC::ClockId::Axi4
                                      : LibXR::SG200XRCC::ClockId::Axi6;
    const uint32_t expected_rate = spi_worker ? 187500000u : 100000000u;
    const uint32_t expected_bus_rate = spi_worker ? 300000000u : 100000000u;
    if (EnableRccClockWithRetry(clock) != LibXR::ErrorCode::OK ||
        EnableRccClockWithRetry(bus_clock) != LibXR::ErrorCode::OK)
    {
      errors |= 1u << 18u;
    }
    if (!RccRateMatchesWithRetry(clock, expected_rate) ||
        !RccRateMatchesWithRetry(bus_clock, expected_bus_rate))
    {
      errors |= 1u << 19u;
    }
    if ((iteration & 0xFFu) == 0u &&
        (rcc.EnableClock(LibXR::SG200XRCC::ClockId::None) !=
             LibXR::ErrorCode::ARG_ERR ||
         rcc.ReleaseReset(static_cast<LibXR::SG200XRCC::ResetId>(0xFFFFu)) !=
             LibXR::ErrorCode::ARG_ERR))
    {
      errors |= 1u << 20u;
    }
    rcc_test_iterations.fetch_add(1u, std::memory_order_relaxed);
    if ((iteration & 0x1Fu) == 0x1Fu)
    {
      LibXR::Thread::Yield();
    }
  }
  rcc_test_errors.fetch_or(errors, std::memory_order_acq_rel);

  if (rcc_test_done.fetch_add(1u, std::memory_order_acq_rel) + 1u ==
      RCC_TEST_WORKERS)
  {
    if (rcc_test_contentions.load(std::memory_order_acquire) == 0u)
    {
      rcc_test_errors.fetch_or(1u << 18u, std::memory_order_acq_rel);
    }
    rcc_test_errors.fetch_or(ValidateRccFinalState(), std::memory_order_acq_rel);
    const uint32_t final_errors = rcc_test_errors.load(std::memory_order_acquire);
    PublishRccTestDiagnostics(final_errors);
    PublishRccTestResult(final_errors == 0u ? 0u : 1u, final_errors);
  }
  for (;;)
  {
    LibXR::Thread::Sleep(1000u);
  }
}

void StartRccTest()
{
  PublishRccTestResult(0xFEu, 0u);
  auto& rcc = LibXR::SG200XRCC::Instance();
  uint32_t errors = 0u;

  const uint32_t reset0_before = LibXR::Register32(RCC_TEST_RESET_BASE);
  const uint32_t reset1_before = LibXR::Register32(RCC_TEST_RESET_BASE + 4u);
  if (rcc.EnableClock(LibXR::SG200XRCC::ClockId::None) !=
      LibXR::ErrorCode::ARG_ERR)
  {
    errors |= 1u << 0u;
  }
  if (rcc.ReleaseReset(static_cast<LibXR::SG200XRCC::ResetId>(0xFFFFu)) !=
      LibXR::ErrorCode::ARG_ERR)
  {
    errors |= 1u << 1u;
  }
  if (rcc.PreparePeripheral(LibXR::SG200XRCC::PeripheralId::Count) !=
      LibXR::ErrorCode::ARG_ERR)
  {
    errors |= 1u << 2u;
  }
  if (rcc.ResetPeripheral(LibXR::SG200XRCC::PeripheralId::Count) !=
      LibXR::ErrorCode::ARG_ERR)
  {
    errors |= 1u << 3u;
  }
  if (rcc.ClockRate(LibXR::SG200XRCC::ClockId::None) != 0u)
  {
    errors |= 1u << 4u;
  }
  if (LibXR::Register32(RCC_TEST_RESET_BASE) != reset0_before ||
      LibXR::Register32(RCC_TEST_RESET_BASE + 4u) != reset1_before)
  {
    errors |= 1u << 17u;
  }

  if (rcc.ClockRate(LibXR::SG200XRCC::ClockId::Oscillator) != 25000000u ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::Fpll) != 1500000000u ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::Mpll) == 0u ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::Tpll) == 0u)
  {
    errors |= 1u << 5u;
  }
  if (rcc.ClockRate(LibXR::SG200XRCC::ClockId::MipiMpll) != 0u ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::A0Pll) != 0u ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::DisplayPll) != 0u)
  {
    errors |= 1u << 6u;
  }

  struct PlannedClock
  {
    LibXR::SG200XRCC::ClockId clock;
    uint32_t rate;
    uint32_t error_bit;
  };
  constexpr PlannedClock planned_clocks[] = {
      {LibXR::SG200XRCC::ClockId::Axi4, 300000000u, 7u},
      {LibXR::SG200XRCC::ClockId::Axi6, 100000000u, 8u},
      {LibXR::SG200XRCC::ClockId::Clock1M, 1000000u, 9u},
      {LibXR::SG200XRCC::ClockId::Spi, 187500000u, 10u},
      {LibXR::SG200XRCC::ClockId::I2c, 100000000u, 11u},
  };
  for (const PlannedClock& planned : planned_clocks)
  {
    if (rcc.EnableClock(planned.clock) != LibXR::ErrorCode::OK ||
        rcc.ClockRate(planned.clock) != planned.rate)
    {
      errors |= 1u << planned.error_bit;
    }
  }

  if (rcc.PreparePeripheral(LibXR::SG200XRCC::PeripheralId::I2c1) !=
          LibXR::ErrorCode::OK ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x00Cu,
                          (1u << 7u) | (1u << 18u),
                          (1u << 7u) | (1u << 18u)) ||
      !RccRegisterMatches(RCC_TEST_RESET_BASE, 1u << 28u, 1u << 28u))
  {
    errors |= 1u << 12u;
  }
  if (rcc.ResetPeripheral(LibXR::SG200XRCC::PeripheralId::Spi2) !=
          LibXR::ErrorCode::OK ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x004u, 1u << 11u,
                          1u << 11u) ||
      !RccRegisterMatches(RCC_TEST_RESET_BASE + 4u, 1u << (42u - 32u),
                          1u << (42u - 32u)))
  {
    errors |= 1u << 23u;
  }

  constexpr uintptr_t C906_REGISTERS[] = {
      RCC_TEST_CLOCK_BASE + 0x010u, RCC_TEST_CLOCK_BASE + 0x034u,
      RCC_TEST_CLOCK_BASE + 0x020u, RCC_TEST_CLOCK_BASE + 0x130u,
      RCC_TEST_CLOCK_BASE + 0x134u,
  };
  uint32_t c906_before[sizeof(C906_REGISTERS) / sizeof(C906_REGISTERS[0])]{};
  for (uint32_t index = 0u; index < sizeof(c906_before) / sizeof(c906_before[0]);
       ++index)
  {
    c906_before[index] = LibXR::Register32(C906_REGISTERS[index]);
  }
  if (rcc.EnableClock(LibXR::SG200XRCC::ClockId::C906_0) !=
      LibXR::ErrorCode::NOT_SUPPORT)
  {
    errors |= 1u << 15u;
  }
  for (uint32_t index = 0u; index < sizeof(c906_before) / sizeof(c906_before[0]);
       ++index)
  {
    if (c906_before[index] != LibXR::Register32(C906_REGISTERS[index]))
    {
      errors |= 1u << 15u;
    }
  }

  // Damage both the divider-valid state and parent route. The next call must
  // observe hardware rather than trusting prior software history and repair it.
  auto& spi_divider = LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x100u);
  spi_divider = (spi_divider & ~(0x3Fu << 16u)) | (7u << 16u);
  spi_divider &= ~LibXR::Bit(3u);
  LibXR::Register32(RCC_TEST_CLOCK_BASE + 0x030u) |= LibXR::Bit(30u);
  asm volatile("fence iorw, iorw" ::: "memory");
  if (rcc.EnableClock(LibXR::SG200XRCC::ClockId::Spi) != LibXR::ErrorCode::OK ||
      rcc.ClockRate(LibXR::SG200XRCC::ClockId::Spi) != 187500000u ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x100u,
                          (0x3Fu << 16u) | LibXR::Bit(3u),
                          (8u << 16u) | LibXR::Bit(3u)) ||
      !RccRegisterMatches(RCC_TEST_CLOCK_BASE + 0x030u, LibXR::Bit(30u), 0u))
  {
    errors |= 1u << 14u;
  }

  constexpr uintptr_t STABLE_REGISTERS[] = {
      RCC_TEST_CLOCK_BASE + 0x004u, RCC_TEST_CLOCK_BASE + 0x008u,
      RCC_TEST_CLOCK_BASE + 0x00Cu,
      RCC_TEST_CLOCK_BASE + 0x030u, RCC_TEST_CLOCK_BASE + 0x0B8u,
      RCC_TEST_CLOCK_BASE + 0x0BCu, RCC_TEST_CLOCK_BASE + 0x0FCu,
      RCC_TEST_CLOCK_BASE + 0x100u, RCC_TEST_CLOCK_BASE + 0x104u,
      RCC_TEST_RESET_BASE, RCC_TEST_RESET_BASE + 4u,
  };
  uint32_t stable_before[sizeof(STABLE_REGISTERS) / sizeof(STABLE_REGISTERS[0])]{};
  for (uint32_t index = 0u;
       index < sizeof(stable_before) / sizeof(stable_before[0]); ++index)
  {
    stable_before[index] = LibXR::Register32(STABLE_REGISTERS[index]);
  }
  for (uint32_t iteration = 0u; iteration < 512u; ++iteration)
  {
    for (const PlannedClock& planned : planned_clocks)
    {
      if (rcc.EnableClock(planned.clock) != LibXR::ErrorCode::OK)
      {
        errors |= 1u << 13u;
      }
    }
  }
  for (uint32_t index = 0u;
       index < sizeof(stable_before) / sizeof(stable_before[0]); ++index)
  {
    if (stable_before[index] != LibXR::Register32(STABLE_REGISTERS[index]))
    {
      errors |= 1u << 13u;
    }
  }
  if (ValidateRccFinalState() != 0u)
  {
    errors |= 1u << 13u;
  }

  if (rcc.ReleaseReset(LibXR::SG200XRCC::ResetId::I2c1) !=
          LibXR::ErrorCode::OK ||
      !RccRegisterMatches(RCC_TEST_RESET_BASE, 1u << 28u, 1u << 28u))
  {
    errors |= 1u << 16u;
  }
  rcc_test_errors.store(errors, std::memory_order_release);
  PublishRccTestDiagnostics(errors);
  PublishRccTestResult(0xFDu, errors);
  for (uint32_t remaining = RCC_TEST_WORKERS; remaining > 0u; --remaining)
  {
    const uint32_t worker = remaining - 1u;
    rcc_test_workers[worker].Create<uint32_t>(
        worker, RccTestWorker, "rcc", 2048u,
        worker == 0u ? LibXR::Thread::Priority::LOW
                     : LibXR::Thread::Priority::HIGH);
  }
}
#endif

#ifdef SG200X_TIMEBASE_TEST
constexpr uint32_t TIMEBASE_TEST_MAGIC = 0x54424D50u;  // "TBMP"

void StartTimebaseTest(void*)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  const uint64_t start = static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
  trace[16] = TIMEBASE_TEST_MAGIC;
  trace[17] = LibXR::SG200XTimebase::ClockHz();
  trace[18] = static_cast<uint32_t>(start);
  trace[19] = static_cast<uint32_t>(start >> 32u);
  trace[20] = 0u;  // zero means the delay is in progress
  flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 20u);

  LibXR::Timebase::DelayMicroseconds(1000u);

  const uint64_t end = static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
  trace[20] = 1u;
  trace[21] = static_cast<uint32_t>(end);
  trace[22] = static_cast<uint32_t>(end >> 32u);
  trace[23] = static_cast<uint32_t>(end - start);
  trace[24] = static_cast<uint32_t>((end - start) >> 32u);
  flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 36u);
}
#endif

#ifdef SG200X_ATOMIC_TEST
// This is a C906L-local test.  The separate Linux stress tool verifies the
// other C906 hart under OS preemption; it must not be confused with a
// cache-coherent cross-processor test because the two processors' caches are
// explicitly managed by the SG2002 shared-memory transport.
constexpr uint32_t ATOMIC_TEST_WORKERS = 2u;
constexpr uint32_t ATOMIC_TEST_ITERATIONS = 20000u;
constexpr uint32_t ATOMIC_TEST_DIRECT_ITERATIONS = 4096u;
constexpr uint32_t ATOMIC_TEST_TRACE_EVENT = 1u;  // CVITEK_BOOT_TRACE_EVENT_IRQ_RESULT

static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "C906L uint32_t atomics must not fall back to a lock library");
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "C906L uint64_t atomics must not fall back to a lock library");
static_assert(std::atomic<bool>::is_always_lock_free,
              "C906L bool atomics must not fall back to a lock library");
static_assert(std::atomic<uint8_t>::is_always_lock_free,
              "C906L uint8_t atomics must not fall back to a lock library");
static_assert(std::atomic<uint16_t>::is_always_lock_free,
              "C906L uint16_t atomics must not fall back to a lock library");
static_assert(std::atomic<uintptr_t>::is_always_lock_free,
              "C906L pointer-width atomics must not fall back to a lock library");

alignas(64) volatile uint32_t atomic_test_direct{0u};
alignas(64) volatile uint32_t atomic_test_global_probe{0u};
alignas(64) std::atomic<uint32_t> atomic_test_counter{0u};
alignas(64) std::atomic<uint64_t> atomic_test_counter64{0u};
alignas(64) std::atomic<uint8_t> atomic_test_counter8{0u};
alignas(64) std::atomic<uint16_t> atomic_test_counter16{0u};
alignas(64) std::atomic<bool> atomic_test_flag{false};
alignas(64) std::atomic<uintptr_t> atomic_test_pointer{0u};
alignas(64) std::atomic<uint32_t> atomic_test_done{0u};
alignas(64) std::atomic<uint32_t> atomic_test_errors{0u};
LibXR::Thread atomic_test_workers[ATOMIC_TEST_WORKERS];

uint32_t AtomicFetchAdd(volatile uint32_t* address, uint32_t increment)
{
  uint32_t previous = 0u;
  // Keep a direct AMO in the test in addition to std::atomic.  This catches
  // an illegal-instruction trap or a non-functional AMO implementation rather
  // than merely proving that the C++ source compiled.
  asm volatile("amoadd.w.aqrl %0, %2, (%1)"
               : "=&r"(previous)
               : "r"(address), "r"(increment)
               : "memory");
  return previous;
}

void PublishAtomicTestResult(uint32_t result, uint32_t detail)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  uint32_t sequence = trace[8];
  uint32_t seen = trace[14];
  if ((sequence ^ trace[9]) != 0xFFFFFFFFu)
  {
    sequence = 0u;
  }
  if ((seen ^ trace[15]) != 0xFFFFFFFFu)
  {
    seen = 0u;
  }
  seen |= 1u << (ATOMIC_TEST_TRACE_EVENT - 1u);
  const uint32_t argument = (result << 24u) | (detail & 0x00FFFFFFu);
  trace[12] = argument;
  trace[13] = ~argument;
  trace[14] = seen;
  trace[15] = ~seen;
  trace[10] = ATOMIC_TEST_TRACE_EVENT;
  trace[11] = ~trace[10];
  ++sequence;
  trace[8] = sequence;
  trace[9] = ~sequence;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 32u, 32u);
}

void AtomicTestWorker(uint32_t arg)
{
  const uint32_t worker_index = static_cast<uint32_t>(arg);
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  const uint32_t stage_index = 16u + worker_index * 16u;
  trace[stage_index] = 1u;
  flush_dcache_range(BOOT_TRACE_ADDRESS + stage_index * 4u, 4u);
  if (!atomic_test_counter.is_lock_free())
  {
    atomic_test_errors.fetch_or(1u, std::memory_order_relaxed);
  }
  for (uint32_t index = 0u; index < ATOMIC_TEST_ITERATIONS; ++index)
  {
    atomic_test_counter.fetch_add(1u, std::memory_order_seq_cst);
    if ((index & 31u) == 31u)
    {
      LibXR::Thread::Yield();
    }
  }
  trace[stage_index] = 2u;
  flush_dcache_range(BOOT_TRACE_ADDRESS + stage_index * 4u, 4u);
  for (uint32_t index = 0u; index < ATOMIC_TEST_ITERATIONS; ++index)
  {
    uint64_t expected = atomic_test_counter64.load(std::memory_order_relaxed);
    while (!atomic_test_counter64.compare_exchange_weak(
        expected, expected + 1u, std::memory_order_seq_cst, std::memory_order_relaxed))
    {
    }
    if ((index & 31u) == 31u)
    {
      LibXR::Thread::Yield();
    }
  }
  trace[stage_index] = 3u;
  flush_dcache_range(BOOT_TRACE_ADDRESS + stage_index * 4u, 4u);
  for (uint32_t index = 0u; index < ATOMIC_TEST_ITERATIONS; ++index)
  {
    atomic_test_counter8.fetch_add(1u, std::memory_order_seq_cst);
    atomic_test_counter16.fetch_add(1u, std::memory_order_seq_cst);
    if ((index & 31u) == 31u)
    {
      LibXR::Thread::Yield();
    }
  }
  atomic_test_flag.store(true, std::memory_order_seq_cst);
  atomic_test_pointer.store(reinterpret_cast<uintptr_t>(&atomic_test_counter),
                            std::memory_order_seq_cst);
  trace[stage_index] = 4u;
  flush_dcache_range(BOOT_TRACE_ADDRESS + stage_index * 4u, 4u);
  for (uint32_t index = 0u;
       index < (worker_index == 0u ? ATOMIC_TEST_DIRECT_ITERATIONS : 0u); ++index)
  {
    AtomicFetchAdd(&atomic_test_direct, 1u);
    if ((index & 1023u) == 1023u)
    {
      LibXR::Thread::Yield();
    }
  }
  trace[stage_index] = 5u;
  flush_dcache_range(BOOT_TRACE_ADDRESS + stage_index * 4u, 4u);

  if (atomic_test_done.fetch_add(1u, std::memory_order_acq_rel) + 1u ==
      ATOMIC_TEST_WORKERS)
  {
    const uint32_t expected = ATOMIC_TEST_WORKERS * ATOMIC_TEST_ITERATIONS;
    const uint32_t observed = atomic_test_counter.load(std::memory_order_acquire);
    uint32_t errors = atomic_test_errors.load(std::memory_order_acquire);
    if (observed != expected)
    {
      errors |= 4u;
    }
    if (atomic_test_counter64.load(std::memory_order_acquire) != expected)
    {
      errors |= 8u;
    }
    if (atomic_test_counter8.load(std::memory_order_acquire) !=
            static_cast<uint8_t>(expected) ||
        atomic_test_counter16.load(std::memory_order_acquire) !=
            static_cast<uint16_t>(expected) ||
        !atomic_test_flag.load(std::memory_order_acquire) ||
        atomic_test_pointer.load(std::memory_order_acquire) !=
            reinterpret_cast<uintptr_t>(&atomic_test_counter))
    {
      errors |= 16u;
    }
    if (atomic_test_direct != ATOMIC_TEST_DIRECT_ITERATIONS)
    {
      errors |= 2u;
    }
    PublishAtomicTestResult(errors == 0u ? 0u : errors, observed);
  }
  // Keep the test tasks alive so task teardown/heap reclamation cannot mask
  // the atomic result being published by the last worker.
  for (;;)
  {
    LibXR::Thread::Yield();
  }
}

void StartAtomicTest()
{
  // Distinguish a test that has not started from a zero-valued result left by
  // the remoteproc host sentinel.
  PublishAtomicTestResult(0xFEu, 0u);
  volatile uint32_t direct_probe = 0u;
  if (AtomicFetchAdd(&direct_probe, 1u) != 0u || direct_probe != 1u)
  {
    PublishAtomicTestResult(0xFDu, direct_probe);
    return;
  }
  // Match the official C906 ISA smoke-test shape: first validate one AMO at a
  // statically allocated, cacheable firmware address before involving tasks.
  if (AtomicFetchAdd(&atomic_test_global_probe, 1u) != 0u ||
      atomic_test_global_probe != 1u)
  {
    PublishAtomicTestResult(0xFAu, atomic_test_global_probe);
    return;
  }
  PublishAtomicTestResult(0xF9u, 0u);
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[16] = static_cast<uint32_t>(xPortGetFreeHeapSize());
  flush_dcache_range(BOOT_TRACE_ADDRESS + 64u, 4u);
  for (uint32_t worker = 0u; worker < ATOMIC_TEST_WORKERS; ++worker)
  {
    atomic_test_workers[worker].Create<uint32_t>(
        worker, AtomicTestWorker, "atomic", 2048u,
        static_cast<LibXR::Thread::Priority>(configMAX_PRIORITIES - 2u));
    trace[17u + worker] = static_cast<uint32_t>(xPortGetFreeHeapSize());
    flush_dcache_range(BOOT_TRACE_ADDRESS + (68u + worker * 4u), 4u);
  }
}
#endif

#ifdef SG200X_WATCHDOG_TEST
constexpr uint32_t WATCHDOG_TEST_MARKER = 0x57445432u;
constexpr uint32_t WATCHDOG_TEST_TIMEOUT_MS = 500u;

void PublishWatchdogTestMarker(uint32_t value)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[20] = WATCHDOG_TEST_MARKER;
  trace[21] = ~WATCHDOG_TEST_MARKER;
  trace[22] = value;
  trace[23] = ~value;
  flush_dcache_range(BOOT_TRACE_ADDRESS + 80u, 16u);
}

void StartWatchdogTest(void*)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  const uint32_t boots = ((trace[24] ^ trace[25]) == 0xFFFFFFFFu) ? trace[24] : 0u;
  trace[24] = boots + 1u;
  trace[25] = ~trace[24];
  flush_dcache_range(BOOT_TRACE_ADDRESS + 96u, 8u);
  PublishWatchdogTestMarker(1u);

  static LibXR::SG200XWatchdog watchdog(
      LibXR::SG200XWatchdog::Instance::WDT2, WATCHDOG_TEST_TIMEOUT_MS, 100u,
      LibXR::SG200XWatchdog::DEFAULT_CLOCK_HZ, LibXR::SG200XWatchdog::ResetTarget::CPU,
      LibXR::SG200XWatchdog::ResponseMode::RESET);
  PublishWatchdogTestMarker(watchdog.IsValid() ? 2u : 0xFFu);
}
#endif

struct LedState
{
  LibXR::SG200XGPIO* led;
  bool level;
};

void BlinkLed(LedState* state)
{
  if (state == nullptr || state->led == nullptr)
  {
    return;
  }

  state->level = !state->level;
  state->led->Write(state->level);
}
}  // namespace

extern "C" void sg200x_config_assert(const char* file, unsigned long line)
{
  // configASSERT can run with the scheduler or heap inconsistent. Record only
  // simple data in the remoteproc trace page and halt this C906L instance.
  uint32_t file_tag = 2166136261u;
  while (*file != '\0')
  {
    file_tag = (file_tag ^ static_cast<uint8_t>(*file++)) * 16777619u;
  }

  auto* const trace = reinterpret_cast<volatile uint32_t*>(BOOT_TRACE_ADDRESS);
  trace[4] = 0xC906A55Eu;
  trace[5] = ~trace[4];
  trace[6] = static_cast<uint32_t>(line);
  trace[7] = ~trace[6];
  trace[8] = 0xA55E0001u;
  trace[9] = ~trace[8];
  trace[10] = file_tag;
  trace[11] = ~trace[10];
  flush_dcache_range(BOOT_TRACE_ADDRESS + 16u, 32u);

  for (;;)
  {
    asm volatile("wfi");
  }
}

extern "C" void CreateDefaultTask(void)
{
#ifdef SG200X_DMA_TEST
  dma_start_timer =
      LibXR::Timer::CreateTask<void*>(StartDmaTestDeferred, nullptr, 100u);
  if (dma_start_timer != nullptr)
  {
    LibXR::Timer::Add(dma_start_timer);
    LibXR::Timer::Start(dma_start_timer);
  }
  return;
#elif defined(SG200X_I2C_DMA_TEST)
  i2c_test_thread.Create<void*>(nullptr, StartI2cDmaTestTask, "i2c_test", 2048u,
                                LibXR::Thread::Priority::LOW);
  return;
#elif defined(SG200X_I2C1_INA228_TEST)
  ina228_i2c_test_thread.Create<void*>(nullptr, StartIna228I2c1Test, "ina228_i2c1",
                                      2048u, LibXR::Thread::Priority::LOW);
  return;
#elif defined(SG200X_SPI4_TEST)
  const auto timer = LibXR::Timer::CreateTask<void*>(StartSpi4Test, nullptr, 100u);
  if (timer != nullptr)
  {
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }
  return;
#elif defined(SG200X_SPI2_TEST)
  spi2_test_thread.Create<void*>(nullptr, StartSpi2Test, "spi2_test", 2048u,
                                 LibXR::Thread::Priority::LOW);
  return;
#elif defined(SG200X_SPI2_STRESS_TEST)
  spi2_test_thread.Create<void*>(nullptr, StartSpi2StressTest, "spi2_stress", 2048u,
                                 LibXR::Thread::Priority::LOW);
  return;
#elif defined(SG200X_ADC_TEST)
  const auto timer = LibXR::Timer::CreateTask<void*>(StartAdcTest, nullptr, 1000u);
  if (timer != nullptr)
  {
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }
  return;
#elif defined(SG200X_PWM_ADC_TEST)
  const auto timer =
      LibXR::Timer::CreateTask<void*>(StartPwmAdcTest, nullptr, 250u);
  if (timer != nullptr)
  {
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }
  return;
#elif defined(SG200X_RCC_TEST)
  StartRccTest();
  return;
#elif defined(SG200X_TIMEBASE_TEST)
  const auto timer =
      LibXR::Timer::CreateTask<void*>(StartTimebaseTest, nullptr, 100u);
  if (timer != nullptr)
  {
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }
  return;
#elif defined(SG200X_ATOMIC_TEST)
  StartAtomicTest();
  return;
#elif defined(SG200X_WATCHDOG_TEST)
  const auto timer = LibXR::Timer::CreateTask<void*>(StartWatchdogTest, nullptr, 100u);
  if (timer != nullptr)
  {
    LibXR::Timer::Add(timer);
    LibXR::Timer::Start(timer);
  }
  return;
#else
  // GPIOA_14 is the active-low user LED on LicheeRV Nano.
  static LibXR::SG200XGPIO led(LibXR::SG200XGPIO::Bank::A, 14u);
  const auto config_result =
      led.SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (config_result != LibXR::ErrorCode::OK)
  {
    return;
  }

  led.Write(false);

  static LedState state{&led, false};
  const auto timer = LibXR::Timer::CreateTask<LedState*>(BlinkLed, &state, LED_PERIOD_MS);
  if (timer == nullptr)
  {
    return;
  }
  LibXR::Timer::Add(timer);
  LibXR::Timer::Start(timer);
#endif
}

extern "C" void DefaultTask(void* argument)
{
  BlinkLed(static_cast<LedState*>(argument));
}

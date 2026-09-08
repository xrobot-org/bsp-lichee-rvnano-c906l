#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"
#include "sg200x_ll_csr.h"
#include "sg200x_ll_dma.h"
#include "sg200x_ll_dmamux.h"
#include "sg200x_ll_plic.h"
#include "sg200x_rcc.hpp"

extern "C" int request_irq(unsigned int irqn, int (*handler)(int, void*),
                           unsigned long flags, const char* name, void* argument)
    __attribute__((weak));

namespace LibXR
{

/** SG2002 eight-channel DesignWare AXI DMA controller. */
class SG200XDMAC final
{
 public:
  enum class Request : uint8_t
  {
    NONE = SGLL_DMA_REQUEST_NONE,
    SPI0_RX = SGLL_DMA_REQUEST_SPI0_RX,
    SPI0_TX = SGLL_DMA_REQUEST_SPI0_TX,
    SPI1_RX = SGLL_DMA_REQUEST_SPI1_RX,
    SPI1_TX = SGLL_DMA_REQUEST_SPI1_TX,
    SPI2_RX = SGLL_DMA_REQUEST_SPI2_RX,
    SPI2_TX = SGLL_DMA_REQUEST_SPI2_TX,
    SPI3_RX = SGLL_DMA_REQUEST_SPI3_RX,
    SPI3_TX = SGLL_DMA_REQUEST_SPI3_TX,
    I2C0_RX = SGLL_DMA_REQUEST_I2C0_RX,
    I2C0_TX = SGLL_DMA_REQUEST_I2C0_TX,
    I2C1_RX = SGLL_DMA_REQUEST_I2C1_RX,
    I2C1_TX = SGLL_DMA_REQUEST_I2C1_TX,
    I2C2_RX = SGLL_DMA_REQUEST_I2C2_RX,
    I2C2_TX = SGLL_DMA_REQUEST_I2C2_TX,
    I2C3_RX = SGLL_DMA_REQUEST_I2C3_RX,
    I2C3_TX = SGLL_DMA_REQUEST_I2C3_TX,
    I2C4_RX = SGLL_DMA_REQUEST_I2C4_RX,
    I2C4_TX = SGLL_DMA_REQUEST_I2C4_TX,
  };

  enum class Direction : uint8_t
  {
    MEMORY_TO_MEMORY = SGLL_DMA_MEMORY_TO_MEMORY,
    MEMORY_TO_PERIPHERAL = SGLL_DMA_MEMORY_TO_PERIPHERAL,
    PERIPHERAL_TO_MEMORY = SGLL_DMA_PERIPHERAL_TO_MEMORY,
  };

  enum class Width : uint8_t
  {
    BYTE = SGLL_DMA_WIDTH_BYTE,
    HALF_WORD = SGLL_DMA_WIDTH_HALF_WORD,
  };

  enum class Mode : uint8_t
  {
    NORMAL = SGLL_DMA_MODE_NORMAL,
    CIRCULAR = SGLL_DMA_MODE_CIRCULAR,
  };

  using Callback = void (*)(void*, ErrorCode, bool in_isr);

  struct Transfer
  {
    uintptr_t memory = 0u;
    size_t memory_capacity = 0u;
    uintptr_t peripheral = 0u;
    size_t peripheral_capacity = 0u;
    size_t count = 0u;
    Request request = Request::NONE;
    Direction direction = Direction::MEMORY_TO_PERIPHERAL;
    Width width = Width::BYTE;
    Mode mode = Mode::NORMAL;
    Callback callback = nullptr;
    void* context = nullptr;
  };

  static constexpr uint8_t CHANNEL_COUNT = DMA_CHANNEL_COUNT;
  // Channels 0-3 remain owned by Linux; the C906L adapter owns 4-7.
  static constexpr uint32_t OWNED_CHANNEL_MASK = 0xF0u;
  static constexpr uintptr_t BASE = DMA_BASE;
  static constexpr uint8_t IRQ = IRQ_SDMA;

  static ErrorCode Acquire(uint8_t& channel);
  static ErrorCode AcquireFixed(uint8_t channel);
  static ErrorCode Release(uint8_t channel, bool in_isr = false);
  static ErrorCode Start(uint8_t channel, const Transfer& transfer,
                         DMA_LLI_Type* descriptor = nullptr);
  static ErrorCode Abort(uint8_t channel, bool in_isr = false);

 private:
  static ErrorCode Initialize();
  static int InterruptHandler(int irq, void* argument);
  static void CheckInterrupt(bool in_isr);
  static void CleanForDevice(uintptr_t address, size_t size) noexcept;
  static void PrepareForDeviceWrite(uintptr_t address, size_t size) noexcept;
  static void InvalidateForCpu(uintptr_t address, size_t size) noexcept;
};

namespace detail
{
inline constexpr uint32_t DMA_DISABLE_WAIT_ATTEMPTS = 100000u;

static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "C906L DMA state must not use a library lock across task preemption");

// Inline variables keep this header-only adapter's state program-wide rather
// than creating one anonymous-namespace copy per translation unit.
inline std::atomic<uint32_t> sg200x_dma_allocated{0u};
inline std::atomic<uint32_t> sg200x_dma_armed{0u};

enum class DmaInitializationState : uint8_t
{
  UNINITIALIZED,
  INITIALIZING,
  READY,
};

inline std::atomic<DmaInitializationState> sg200x_dma_initialization_state{
    DmaInitializationState::UNINITIALIZED};
inline std::atomic_flag sg200x_dma_interrupt_dispatching = ATOMIC_FLAG_INIT;

struct Completion
{
  SG200XDMAC::Transfer transfer{};
  uint32_t status = 0u;
  bool pending = false;
};

inline SG200XDMAC::Transfer sg200x_dma_active[SG200XDMAC::CHANNEL_COUNT]{};
inline Completion sg200x_dma_completed[SG200XDMAC::CHANNEL_COUNT]{};
alignas(DMA_LLI_ALIGNMENT) inline DMA_LLI_Type
    sg200x_dma_lli[SG200XDMAC::CHANNEL_COUNT]{};

inline bool WaitForChannelDisabled(uint32_t channels) noexcept
{
  return sgll_dma_channels_wait_disabled(channels, DMA_DISABLE_WAIT_ATTEMPTS);
}

inline constexpr bool IsOwnedChannel(uint8_t channel) noexcept
{
  return channel < SG200XDMAC::CHANNEL_COUNT &&
         (SG200XDMAC::OWNED_CHANNEL_MASK & (1u << channel)) != 0u;
}

inline constexpr size_t AlignUpToCacheLine(size_t size) noexcept
{
  return (size + SGLL_DCACHE_LINE_SIZE - 1u) & ~(size_t{SGLL_DCACHE_LINE_SIZE} - 1u);
}

inline bool OwnsCacheRange(uintptr_t address, size_t size, size_t capacity) noexcept
{
  return (address % SGLL_DCACHE_LINE_SIZE) == 0u &&
         size <= static_cast<size_t>(-1) - (SGLL_DCACHE_LINE_SIZE - 1u) &&
         capacity >= AlignUpToCacheLine(size);
}

inline void EnableMachineExternalInterrupts() noexcept
{
#if defined(__riscv)
  sgll_plic_core_enable();
#endif
}
}  // namespace detail

inline void SG200XDMAC::CleanForDevice(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    sgll_csr_dcache_clean_range(address, size);
  }
}

inline void SG200XDMAC::PrepareForDeviceWrite(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    sgll_csr_dcache_clean_invalidate_range(address, size);
  }
}

inline void SG200XDMAC::InvalidateForCpu(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    sgll_csr_dcache_invalidate_range(address, size);
  }
}

inline ErrorCode SG200XDMAC::Initialize()
{
  detail::DmaInitializationState expected = detail::DmaInitializationState::UNINITIALIZED;
  if (!detail::sg200x_dma_initialization_state.compare_exchange_strong(
          expected, detail::DmaInitializationState::INITIALIZING,
          std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return expected == detail::DmaInitializationState::READY ? ErrorCode::OK
                                                             : ErrorCode::BUSY;
  }

  if (SG200XRCC::Instance().PreparePeripheral(SG200XRCC::PeripheralId::Sdma) !=
      ErrorCode::OK)
  {
    detail::sg200x_dma_initialization_state.store(
        detail::DmaInitializationState::UNINITIALIZED, std::memory_order_release);
    return ErrorCode::STATE_ERR;
  }

  // Retire only C906L-owned channels. Linux channels 0-3 may be active.
  sgll_dma_channels_disable_request(OWNED_CHANNEL_MASK);
  sgll_csr_fence_io();
  if (!detail::WaitForChannelDisabled(OWNED_CHANNEL_MASK))
  {
    sgll_dma_channels_abort(OWNED_CHANNEL_MASK);
    sgll_csr_fence_io();
    if (!detail::WaitForChannelDisabled(OWNED_CHANNEL_MASK))
    {
      detail::sg200x_dma_initialization_state.store(
          detail::DmaInitializationState::UNINITIALIZED, std::memory_order_release);
      return ErrorCode::TIMEOUT;
    }
  }
  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    if (detail::IsOwnedChannel(channel))
    {
      sgll_dma_channel_interrupt_clear(channel, UINT32_MAX);
    }
  }

  for (uint32_t cpu = 0u; cpu < 2u; ++cpu)
  {
    sgll_dmamux_interrupt_route_set(
        cpu, sgll_dmamux_interrupt_route_get(cpu) & ~OWNED_CHANNEL_MASK);
  }
  sgll_dmamux_interrupt_route_set(
      2u, sgll_dmamux_interrupt_route_get(2u) | OWNED_CHANNEL_MASK);
  sgll_dma_enable(true, true);
  if (request_irq == nullptr ||
      request_irq(IRQ, &SG200XDMAC::InterruptHandler, 0u, "sg200x-dma", nullptr) != 0)
  {
    detail::sg200x_dma_initialization_state.store(
        detail::DmaInitializationState::UNINITIALIZED, std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }

  // remoteproc reset does not reset the external PLIC context. Complete a
  // stale source-25 claim after request_irq has installed the source.
  sgll_plic_irq_complete(IRQ);
  sgll_csr_fence_io();
  detail::EnableMachineExternalInterrupts();
  detail::sg200x_dma_initialization_state.store(detail::DmaInitializationState::READY,
                                                std::memory_order_release);
  return ErrorCode::OK;
}

inline ErrorCode SG200XDMAC::Acquire(uint8_t& channel)
{
  const ErrorCode initialized_result = Initialize();
  if (initialized_result != ErrorCode::OK)
  {
    return initialized_result;
  }
  uint32_t used = detail::sg200x_dma_allocated.load(std::memory_order_acquire);
  for (;;)
  {
    for (uint8_t index = 0u; index < CHANNEL_COUNT; ++index)
    {
      const uint32_t bit = 1u << index;
      if ((OWNED_CHANNEL_MASK & bit) != 0u && (used & bit) == 0u)
      {
        const uint32_t desired = used | bit;
        if (detail::sg200x_dma_allocated.compare_exchange_weak(
                used, desired, std::memory_order_acq_rel, std::memory_order_acquire))
        {
          channel = index;
          return ErrorCode::OK;
        }
        break;
      }
    }
    if ((used & OWNED_CHANNEL_MASK) == OWNED_CHANNEL_MASK)
    {
      return ErrorCode::BUSY;
    }
  }
}

inline ErrorCode SG200XDMAC::AcquireFixed(uint8_t channel)
{
  if (!detail::IsOwnedChannel(channel))
  {
    return ErrorCode::ARG_ERR;
  }
  const ErrorCode initialized = Initialize();
  if (initialized != ErrorCode::OK)
  {
    return initialized;
  }
  const uint32_t bit = 1u << channel;
  const uint32_t previous =
      detail::sg200x_dma_allocated.fetch_or(bit, std::memory_order_acq_rel);
  return (previous & bit) == 0u ? ErrorCode::OK : ErrorCode::BUSY;
}

inline ErrorCode SG200XDMAC::Release(uint8_t channel, bool in_isr)
{
  if (!detail::IsOwnedChannel(channel))
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  if (((detail::sg200x_dma_armed.load(std::memory_order_acquire) |
        sgll_dma_enabled_channels_get()) &
       bit) != 0u)
  {
    const ErrorCode abort_result = Abort(channel, in_isr);
    if (abort_result != ErrorCode::OK)
    {
      return abort_result;
    }
  }
  else
  {
    sgll_dma_channel_interrupt_clear(channel, UINT32_MAX);
    detail::sg200x_dma_active[channel] = {};
  }
  detail::sg200x_dma_allocated.fetch_and(~bit, std::memory_order_release);
  return ErrorCode::OK;
}

inline ErrorCode SG200XDMAC::Start(uint8_t channel, const Transfer& transfer,
                                   DMA_LLI_Type* supplied_descriptor)
{
  const bool valid_direction = transfer.direction == Direction::MEMORY_TO_MEMORY ||
                               transfer.direction == Direction::MEMORY_TO_PERIPHERAL ||
                               transfer.direction == Direction::PERIPHERAL_TO_MEMORY;
  const bool valid_width =
      transfer.width == Width::BYTE || transfer.width == Width::HALF_WORD;
  const bool valid_mode =
      transfer.mode == Mode::NORMAL || transfer.mode == Mode::CIRCULAR;
  const uint8_t request = static_cast<uint8_t>(transfer.request);
  const bool valid_request = transfer.direction == Direction::MEMORY_TO_MEMORY ||
                             (request >= static_cast<uint8_t>(Request::SPI0_RX) &&
                              request <= static_cast<uint8_t>(Request::I2C4_TX));
  if (!detail::IsOwnedChannel(channel) || transfer.memory == 0u ||
      transfer.peripheral == 0u || transfer.count == 0u ||
      transfer.count > DMA_BLOCK_TRANSFER_MAX || transfer.callback == nullptr ||
      !valid_direction || !valid_width || !valid_mode || !valid_request)
  {
    return ErrorCode::ARG_ERR;
  }
  if (supplied_descriptor != nullptr &&
      (reinterpret_cast<uintptr_t>(supplied_descriptor) % DMA_LLI_ALIGNMENT) != 0u)
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  if ((detail::sg200x_dma_allocated.load(std::memory_order_acquire) & bit) == 0u)
  {
    return ErrorCode::STATE_ERR;
  }
  if ((detail::sg200x_dma_armed.load(std::memory_order_acquire) & bit) != 0u ||
      (sgll_dma_enabled_channels_get() & bit) != 0u)
  {
    return ErrorCode::BUSY;
  }

  const size_t item_size = size_t{1u} << static_cast<uint8_t>(transfer.width);
  if (transfer.memory % item_size != 0u || transfer.peripheral % item_size != 0u)
  {
    return ErrorCode::ARG_ERR;
  }
  const size_t bytes = transfer.count << static_cast<uint8_t>(transfer.width);
  if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY &&
      !detail::OwnsCacheRange(transfer.memory, bytes, transfer.memory_capacity))
  {
    return ErrorCode::ARG_ERR;
  }
  if (transfer.direction == Direction::MEMORY_TO_MEMORY &&
      !detail::OwnsCacheRange(transfer.peripheral, bytes, transfer.peripheral_capacity))
  {
    return ErrorCode::ARG_ERR;
  }
  DMA_LLI_Type& descriptor = supplied_descriptor != nullptr
                                 ? *supplied_descriptor
                                 : detail::sg200x_dma_lli[channel];
  const auto direction = static_cast<sgll_dma_direction_t>(transfer.direction);
  const auto width = static_cast<sgll_dma_width_t>(transfer.width);
  const auto mode = static_cast<sgll_dma_mode_t>(transfer.mode);
  const bool receive = transfer.direction == Direction::PERIPHERAL_TO_MEMORY;
  if (!sgll_dma_lli_build(
          &descriptor, receive ? transfer.peripheral : transfer.memory,
          receive ? transfer.memory : transfer.peripheral,
          static_cast<uint32_t>(transfer.count), direction, width, mode,
          mode == SGLL_DMA_MODE_CIRCULAR ? reinterpret_cast<uintptr_t>(&descriptor) : 0u))
  {
    return ErrorCode::ARG_ERR;
  }

  if (transfer.direction == Direction::MEMORY_TO_PERIPHERAL)
  {
    CleanForDevice(transfer.memory, bytes);
  }
  else if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY)
  {
    PrepareForDeviceWrite(transfer.memory, bytes);
  }
  else
  {
    CleanForDevice(transfer.memory, bytes);
    PrepareForDeviceWrite(transfer.peripheral, bytes);
  }

  if (transfer.direction != Direction::MEMORY_TO_MEMORY)
  {
    sgll_dma_request_route_set(channel, request);
  }
  sgll_dma_channel_interrupt_clear(channel, UINT32_MAX);

  CleanForDevice(reinterpret_cast<uintptr_t>(&descriptor), sizeof(descriptor));

  sgll_dma_channel_configure(channel, sgll_dma_config_build(direction, channel),
                             reinterpret_cast<uintptr_t>(&descriptor));
  sgll_dma_channel_interrupt_configure(channel, sgll_dma_interrupt_mask(mode));
  detail::sg200x_dma_active[channel] = transfer;
  sgll_csr_fence_io();
  detail::sg200x_dma_armed.fetch_or(bit, std::memory_order_release);
  sgll_dma_channel_enable(channel);
  sgll_csr_fence_io();
  return ErrorCode::OK;
}

inline ErrorCode SG200XDMAC::Abort(uint8_t channel, bool)
{
  if (!detail::IsOwnedChannel(channel))
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  detail::sg200x_dma_armed.fetch_and(~bit, std::memory_order_acq_rel);
  sgll_dma_channel_disable_request(channel);
  sgll_csr_fence_io();
  if (!detail::WaitForChannelDisabled(bit))
  {
    sgll_dma_channel_abort(channel);
    sgll_csr_fence_io();
    if (!detail::WaitForChannelDisabled(bit))
    {
      detail::sg200x_dma_armed.fetch_or(bit, std::memory_order_release);
      return ErrorCode::TIMEOUT;
    }
  }
  sgll_dma_channel_interrupt_clear(channel, UINT32_MAX);
  detail::sg200x_dma_active[channel] = {};
  return ErrorCode::OK;
}

inline int SG200XDMAC::InterruptHandler(int, void*)
{
  CheckInterrupt(true);
  return 0;
}

inline void SG200XDMAC::CheckInterrupt(bool in_isr)
{
  if (detail::sg200x_dma_interrupt_dispatching.test_and_set(std::memory_order_acquire))
  {
    return;
  }
  sgll_csr_fence_io();
  for (detail::Completion& completion : detail::sg200x_dma_completed)
  {
    completion = {};
  }

  const uint32_t armed_channels =
      detail::sg200x_dma_armed.load(std::memory_order_acquire);
  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    const uint32_t bit = 1u << channel;
    if ((armed_channels & bit) == 0u)
    {
      continue;
    }
    const uint32_t status = sgll_dma_channel_interrupt_status_get(channel);
    if (status == 0u)
    {
      continue;
    }
    sgll_dma_channel_interrupt_clear(channel, status);
    detail::Completion& completion = detail::sg200x_dma_completed[channel];
    completion.transfer = detail::sg200x_dma_active[channel];
    completion.status = status;
    completion.pending = true;
    const bool error = (status & DMA_INT_ERROR_MASK) != 0u;
    if (!error && completion.transfer.mode == Mode::NORMAL)
    {
      detail::sg200x_dma_active[channel] = {};
      detail::sg200x_dma_armed.fetch_and(~bit, std::memory_order_acq_rel);
    }
    if (error && completion.transfer.callback != nullptr)
    {
      sgll_dma_channel_abort(channel);
    }
  }

  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    const detail::Completion& completion = detail::sg200x_dma_completed[channel];
    const Transfer& transfer = completion.transfer;
    if (!completion.pending || transfer.callback == nullptr)
    {
      continue;
    }
    const ErrorCode result = (completion.status & DMA_INT_ERROR_MASK) == 0u
                                 ? ErrorCode::OK
                                 : ErrorCode::FAILED;
    if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY)
    {
      InvalidateForCpu(transfer.memory,
                       transfer.count << static_cast<uint8_t>(transfer.width));
    }
    else if (transfer.direction == Direction::MEMORY_TO_MEMORY)
    {
      InvalidateForCpu(transfer.peripheral,
                       transfer.count << static_cast<uint8_t>(transfer.width));
    }
    transfer.callback(transfer.context, result, in_isr);
  }
  detail::sg200x_dma_interrupt_dispatching.clear(std::memory_order_release);
}

}  // namespace LibXR

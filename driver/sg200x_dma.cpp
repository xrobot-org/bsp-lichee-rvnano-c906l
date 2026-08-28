#include "sg200x_dma.hpp"

#include "sg200x_mmio.hpp"
#include "sg200x_rcc.hpp"

extern "C"
{
#include "arch_helpers.h"
}

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
constexpr uintptr_t TOP_BASE = 0x03000000u;
constexpr uintptr_t DMA_REMAP0 = TOP_BASE + 0x154u;
constexpr uintptr_t DMA_REMAP1 = TOP_BASE + 0x158u;
constexpr uintptr_t DMA_INTERRUPT_MUX = TOP_BASE + 0x298u;
constexpr uintptr_t PLIC_CLAIM_COMPLETE = 0x70200004u;
constexpr uint32_t DMA_INTERRUPT_MUX_CPU0_SHIFT = 0u;
constexpr uint32_t DMA_INTERRUPT_MUX_CPU1_SHIFT = 10u;
constexpr uint32_t DMA_INTERRUPT_MUX_CPU2_SHIFT = 20u;
constexpr uint32_t DMA_INTERRUPT_MUX_CPU2_MASK =
    0x1FFu << DMA_INTERRUPT_MUX_CPU2_SHIFT;
constexpr uint32_t DMA_INTERRUPT_MUX_OWNED_OTHER_CPUS_MASK =
    (SG200XDMAC::OWNED_CHANNEL_MASK << DMA_INTERRUPT_MUX_CPU0_SHIFT) |
    (SG200XDMAC::OWNED_CHANNEL_MASK << DMA_INTERRUPT_MUX_CPU1_SHIFT);
constexpr uint32_t DMA_INTERRUPT_MUX_OWNED_CPU2 =
    SG200XDMAC::OWNED_CHANNEL_MASK << DMA_INTERRUPT_MUX_CPU2_SHIFT;
constexpr uint32_t REG_CFG = 0x10u;
constexpr uint32_t CFG_DMAC_ENABLE = 1u << 0u;
constexpr uint32_t CFG_INTERRUPT_ENABLE = 1u << 1u;
constexpr uint32_t REG_CHEN = 0x18u;
constexpr uint32_t REG_CHABORT = 0x28u;
constexpr uint32_t CHANNEL_BASE = 0x100u;
constexpr uint32_t CHANNEL_STRIDE = 0x100u;
constexpr uint32_t CH_CFG = 0x20u;
constexpr uint32_t CH_LLP = 0x28u;
constexpr uint32_t CH_INTSTATUS_EN = 0x80u;
constexpr uint32_t CH_INTSTATUS = 0x88u;
constexpr uint32_t CH_INTSIGNAL_EN = 0x90u;
constexpr uint32_t CH_INTCLEAR = 0x98u;
constexpr uint64_t CTL_SMS = 1ull << 0u;
constexpr uint64_t CTL_DMS = 1ull << 2u;
constexpr uint64_t CTL_SINC = 1ull << 4u;
constexpr uint64_t CTL_DINC = 1ull << 6u;
constexpr uint32_t CTL_SRC_WIDTH_SHIFT = 8u;
constexpr uint32_t CTL_DST_WIDTH_SHIFT = 11u;
constexpr uint64_t CTL_IOC_BLOCK = 1ull << 58u;
constexpr uint64_t CTL_LLI_LAST = 1ull << 62u;
constexpr uint64_t CTL_LLI_VALID = 1ull << 63u;
constexpr uint32_t CFG_TTFC_SHIFT = 32u;
constexpr uint32_t CFG_SRC_PER_SHIFT = 39u;
constexpr uint32_t CFG_DST_PER_SHIFT = 44u;
constexpr uint32_t CFG_PRIORITY_SHIFT = 49u;
constexpr uint32_t CFG_SRC_OSR_SHIFT = 55u;
constexpr uint32_t CFG_DST_OSR_SHIFT = 59u;
constexpr uint64_t CFG_SRC_MULTIBLK_LLI = 3ull << 0u;
constexpr uint64_t CFG_DST_MULTIBLK_LLI = 3ull << 2u;
constexpr uint32_t INT_BLOCK_DONE = 1u << 0u;
constexpr uint32_t INT_TRANSFER_DONE = 1u << 1u;
constexpr uint32_t INT_ERROR_MASK = 0xF8FF7FE0u;
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "C906L DMA state must not use a library lock across task preemption");
std::atomic<uint32_t> allocated{0u};
// A transfer record is published before CHEN is written so a real hardware
// completion cannot observe an empty callback.
std::atomic<uint32_t> armed{0u};
enum class InitializationState : uint8_t
{
  UNINITIALIZED,
  INITIALIZING,
  READY,
};
std::atomic<InitializationState> initialization_state{InitializationState::UNINITIALIZED};
std::atomic_flag interrupt_dispatching = ATOMIC_FLAG_INIT;
SG200XDMAC::Transfer active[SG200XDMAC::CHANNEL_COUNT]{};

struct Completion
{
  SG200XDMAC::Transfer transfer{};
  uint32_t status = 0u;
  bool pending = false;
};

// The dispatch gate below makes this shared completion workspace
// single-consumer if a second PLIC notification arrives during a callback.
Completion completed[SG200XDMAC::CHANNEL_COUNT]{};

void EnableMachineExternalInterrupts()
{
  constexpr uintptr_t MACHINE_EXTERNAL_INTERRUPT_ENABLE = 1u << 11u;
  asm volatile("csrs mie, %0" : : "r"(MACHINE_EXTERNAL_INTERRUPT_ENABLE) : "memory");
}

// The SG200x SDK programs this DW_axi_dmac through LLI entries, even for a
// one-block I2C transfer. Keep one cache-line-sized entry per claimed channel
// so every data path follows that documented hardware mode.
struct alignas(64) Lli
{
  uint64_t source;
  uint64_t destination;
  uint64_t block_ts;
  uint64_t next;
  uint32_t control_low;
  uint32_t control_high;
  uint32_t source_status;
  uint32_t destination_status;
  uint32_t status_low;
  uint32_t status_high;
  uint32_t reserved_low;
  uint32_t reserved_high;
};
static_assert(sizeof(Lli) == 64u);
Lli lli[SG200XDMAC::CHANNEL_COUNT]{};

uintptr_t ChannelAddress(uint8_t channel, uint32_t offset)
{
  return SG200XDMAC::BASE + CHANNEL_BASE +
         static_cast<uintptr_t>(channel) * CHANNEL_STRIDE + offset;
}

bool WaitForChannelDisabled(uint32_t bit)
{
  constexpr uint32_t DISABLE_WAIT_ATTEMPTS = 100000u;
  for (uint32_t attempt = 0u; attempt < DISABLE_WAIT_ATTEMPTS; ++attempt)
  {
    if ((Register32(SG200XDMAC::BASE + REG_CHEN) & bit) == 0u)
    {
      return true;
    }
  }
  return false;
}

[[nodiscard]] constexpr bool IsOwnedChannel(uint8_t channel) noexcept
{
  return channel < SG200XDMAC::CHANNEL_COUNT &&
         (SG200XDMAC::OWNED_CHANNEL_MASK & (1u << channel)) != 0u;
}

[[nodiscard]] constexpr size_t AlignUpToCacheLine(size_t size) noexcept
{
  return (size + HW_CACHE_LINE_SIZE - 1u) & ~(HW_CACHE_LINE_SIZE - 1u);
}

[[nodiscard]] bool OwnsCacheRange(uintptr_t address, size_t size,
                                  size_t capacity) noexcept
{
  return (address % HW_CACHE_LINE_SIZE) == 0u &&
         size <= static_cast<size_t>(-1) - (HW_CACHE_LINE_SIZE - 1u) &&
         capacity >= AlignUpToCacheLine(size);
}
}  // namespace

void SG200XDMAC::CleanForDevice(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    clean_dcache_range(address, size);
  }
}

void SG200XDMAC::PrepareForDeviceWrite(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    // Preserve bytes outside a short transfer in its first/last cache line,
    // then remove every destination line so no dirty eviction can race DMA.
    flush_dcache_range(address, size);
  }
}

void SG200XDMAC::InvalidateForCpu(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    inv_dcache_range(address, size);
  }
}

ErrorCode SG200XDMAC::Initialize()
{
  InitializationState expected = InitializationState::UNINITIALIZED;
  if (!initialization_state.compare_exchange_strong(
          expected, InitializationState::INITIALIZING, std::memory_order_acq_rel,
          std::memory_order_acquire))
  {
    return expected == InitializationState::READY ? ErrorCode::OK : ErrorCode::BUSY;
  }

  // Linux and C906L share one SDMA controller. Enable its clock/reset path,
  // but never pulse the global reset because Linux can have live channels.
  if (SG200XRCC::Instance().PreparePeripheral(SG200XRCC::PeripheralId::Sdma) !=
      ErrorCode::OK)
  {
    initialization_state.store(InitializationState::UNINITIALIZED,
                               std::memory_order_release);
    return ErrorCode::STATE_ERR;
  }
  // A stopped remoteproc image can leave only its partition active. Retire
  // those channels without disturbing Linux channels 0-3.
  Register32(BASE + REG_CHEN) = OWNED_CHANNEL_MASK << 8u;
  asm volatile("fence iorw, iorw" ::: "memory");
  if (!WaitForChannelDisabled(OWNED_CHANNEL_MASK))
  {
    Register32(BASE + REG_CHABORT) =
        OWNED_CHANNEL_MASK | (OWNED_CHANNEL_MASK << 8u);
    asm volatile("fence iorw, iorw" ::: "memory");
    if (!WaitForChannelDisabled(OWNED_CHANNEL_MASK))
    {
      initialization_state.store(InitializationState::UNINITIALIZED,
                                 std::memory_order_release);
      return ErrorCode::TIMEOUT;
    }
  }
  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    if (IsOwnedChannel(channel))
    {
      Register32(ChannelAddress(channel, CH_INTCLEAR)) = 0xFFFFFFFFu;
    }
  }

  // Remove C906L-owned channels from the other CPU routes and expose only
  // those channels to CPU2. Preserve Linux channels and the common IRQ bit.
  Register32(DMA_INTERRUPT_MUX) =
      (Register32(DMA_INTERRUPT_MUX) &
       ~(DMA_INTERRUPT_MUX_OWNED_OTHER_CPUS_MASK | DMA_INTERRUPT_MUX_CPU2_MASK)) |
      DMA_INTERRUPT_MUX_OWNED_CPU2;
  Register32(BASE + REG_CFG) |= CFG_DMAC_ENABLE | CFG_INTERRUPT_ENABLE;
  if (request_irq == nullptr ||
      request_irq(IRQ, &SG200XDMAC::InterruptHandler, 0u, "sg200x-dma", nullptr) != 0)
  {
    initialization_state.store(InitializationState::UNINITIALIZED,
                               std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }
  // remoteproc reset does not reset the external PLIC context. Complete a
  // source 25 claim that a previously stopped image may have left in service.
  // The source must be enabled by request_irq() before this PLIC accepts EOI.
  Register32(PLIC_CLAIM_COMPLETE) = IRQ;
  asm volatile("fence iorw, iorw" ::: "memory");
  // irq_init() enables global MSTATUS.MIE but deliberately leaves MEIE to
  // drivers. request_irq() has now installed the handler and unmasked PLIC
  // source 25, so no unhandled SDMA completion can trap the C906L.
  EnableMachineExternalInterrupts();
  initialization_state.store(InitializationState::READY, std::memory_order_release);
  return ErrorCode::OK;
}

ErrorCode SG200XDMAC::Acquire(uint8_t& channel)
{
  const ErrorCode initialized_result = Initialize();
  if (initialized_result != ErrorCode::OK)
  {
    return initialized_result;
  }
  uint32_t used = allocated.load(std::memory_order_acquire);
  for (;;)
  {
    for (uint8_t index = 0u; index < CHANNEL_COUNT; ++index)
    {
      const uint32_t bit = 1u << index;
      if ((OWNED_CHANNEL_MASK & bit) != 0u && (used & bit) == 0u)
      {
        const uint32_t desired = used | bit;
        if (allocated.compare_exchange_weak(used, desired, std::memory_order_acq_rel,
                                            std::memory_order_acquire))
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

ErrorCode SG200XDMAC::Release(uint8_t channel, bool in_isr)
{
  if (!IsOwnedChannel(channel))
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  // Normal completion detaches the software record before its callback. The
  // hardware CHEN bit may still read high briefly at that point and must not
  // be mistaken for an in-flight transfer. Circular and cancelled transfers
  // remain armed until their explicit stop path reaches here.
  if ((armed.load(std::memory_order_acquire) & bit) != 0u)
  {
    const ErrorCode abort_result = Abort(channel, in_isr);
    if (abort_result != ErrorCode::OK)
    {
      return abort_result;
    }
  }
  else
  {
    Register32(ChannelAddress(channel, CH_INTCLEAR)) = 0xFFFFFFFFu;
    active[channel] = {};
  }
  allocated.fetch_and(~bit, std::memory_order_release);
  return ErrorCode::OK;
}

ErrorCode SG200XDMAC::Start(uint8_t channel, const Transfer& transfer)
{
  const bool valid_direction = transfer.direction == Direction::MEMORY_TO_MEMORY ||
                               transfer.direction == Direction::MEMORY_TO_PERIPHERAL ||
                               transfer.direction == Direction::PERIPHERAL_TO_MEMORY;
  const bool valid_width =
      transfer.width == Width::BYTE || transfer.width == Width::HALF_WORD;
  const bool valid_mode = transfer.mode == Mode::NORMAL || transfer.mode == Mode::CIRCULAR;
  const uint8_t request = static_cast<uint8_t>(transfer.request);
  const bool valid_request =
      transfer.direction == Direction::MEMORY_TO_MEMORY ||
      (request >= static_cast<uint8_t>(Request::SPI0_RX) &&
       request <= static_cast<uint8_t>(Request::I2C4_TX));
  if (!IsOwnedChannel(channel) || transfer.memory == 0u || transfer.peripheral == 0u ||
      transfer.count == 0u || transfer.count > 0x400000u || transfer.callback == nullptr ||
      !valid_direction || !valid_width || !valid_mode || !valid_request)
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  if ((allocated.load(std::memory_order_acquire) & bit) == 0u)
  {
    return ErrorCode::STATE_ERR;
  }
  if ((armed.load(std::memory_order_acquire) & bit) != 0u ||
      (Register32(BASE + REG_CHEN) & bit) != 0u)
  {
    return ErrorCode::BUSY;
  }

  const size_t bytes = transfer.count << static_cast<uint8_t>(transfer.width);
  if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY &&
      !OwnsCacheRange(transfer.memory, bytes, transfer.memory_capacity))
  {
    return ErrorCode::ARG_ERR;
  }
  if (transfer.direction == Direction::MEMORY_TO_MEMORY &&
      !OwnsCacheRange(transfer.peripheral, bytes, transfer.peripheral_capacity))
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
    const uintptr_t remap = channel < 4u ? DMA_REMAP0 : DMA_REMAP1;
    const uint32_t field_shift = static_cast<uint32_t>(channel % 4u) * 8u;
    uint32_t mapping = Register32(remap);
    mapping = (mapping & ~(0x3Fu << field_shift)) |
              (static_cast<uint32_t>(transfer.request) << field_shift) | (1u << 31u);
    Register32(remap) = mapping;
  }
  const uintptr_t ch = ChannelAddress(channel, 0u);
  Register32(ch + CH_INTCLEAR) = 0xFFFFFFFFu;
  Lli& descriptor = lli[channel];
  descriptor = {};
  if (transfer.direction == Direction::MEMORY_TO_PERIPHERAL ||
      transfer.direction == Direction::MEMORY_TO_MEMORY)
  {
    descriptor.source = transfer.memory;
    descriptor.destination = transfer.peripheral;
  }
  else
  {
    descriptor.source = transfer.peripheral;
    descriptor.destination = transfer.memory;
  }
  descriptor.block_ts = transfer.count - 1u;
  uint64_t control = CTL_IOC_BLOCK;
  control |= CTL_SMS | CTL_DMS;
  control |= static_cast<uint64_t>(transfer.width) << CTL_SRC_WIDTH_SHIFT;
  control |= static_cast<uint64_t>(transfer.width) << CTL_DST_WIDTH_SHIFT;
  if (transfer.direction == Direction::MEMORY_TO_PERIPHERAL)
  {
    control |= CTL_DINC;
  }
  else if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY)
  {
    control |= CTL_SINC;
  }
  if (transfer.mode == Mode::CIRCULAR)
  {
    descriptor.next = reinterpret_cast<uintptr_t>(&descriptor);
    control |= CTL_LLI_VALID;
  }
  else
  {
    control |= CTL_LLI_LAST | CTL_LLI_VALID;
  }
  descriptor.control_low = static_cast<uint32_t>(control);
  descriptor.control_high = static_cast<uint32_t>(control >> 32u);
  uint64_t cfg = CFG_SRC_MULTIBLK_LLI | CFG_DST_MULTIBLK_LLI;
  cfg |= 7ull << CFG_PRIORITY_SHIFT;
  cfg |= 15ull << CFG_SRC_OSR_SHIFT;
  cfg |= 15ull << CFG_DST_OSR_SHIFT;
  if (transfer.direction == Direction::MEMORY_TO_PERIPHERAL)
  {
    cfg |= 1ull << CFG_TTFC_SHIFT;
    cfg |= static_cast<uint64_t>(channel) << CFG_SRC_PER_SHIFT;
    cfg |= static_cast<uint64_t>(channel) << CFG_DST_PER_SHIFT;
  }
  else if (transfer.direction == Direction::PERIPHERAL_TO_MEMORY)
  {
    cfg |= 2ull << CFG_TTFC_SHIFT;
    cfg |= static_cast<uint64_t>(channel) << CFG_SRC_PER_SHIFT;
    cfg |= static_cast<uint64_t>(channel) << CFG_DST_PER_SHIFT;
  }
  CleanForDevice(reinterpret_cast<uintptr_t>(&descriptor), sizeof(descriptor));
  Register32(ch + CH_CFG) = static_cast<uint32_t>(cfg);
  Register32(ch + CH_CFG + 4u) = static_cast<uint32_t>(cfg >> 32u);
  Register32(ch + CH_LLP) = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&descriptor));
  Register32(ch + CH_LLP + 4u) =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&descriptor) >> 32u);
  const uint32_t interrupt_mask = INT_BLOCK_DONE | INT_ERROR_MASK |
                                  (transfer.mode == Mode::NORMAL ? INT_TRANSFER_DONE : 0u);
  Register32(ch + CH_INTSTATUS_EN) = interrupt_mask;
  Register32(ch + CH_INTSIGNAL_EN) = interrupt_mask;
  active[channel] = transfer;
  // Publish the software completion record and descriptor before arming the
  // channel.  The DMA engine may complete a short transfer immediately; the
  // IRQ handler must never observe an empty active[] entry in that case.
  asm volatile("fence rw, rw" ::: "memory");
  armed.fetch_or(bit, std::memory_order_release);
  Register32(BASE + REG_CHEN) =
      static_cast<uint32_t>(bit) | (static_cast<uint32_t>(bit) << 8u);
  asm volatile("fence rw, rw" ::: "memory");
  return ErrorCode::OK;
}

ErrorCode SG200XDMAC::Abort(uint8_t channel, bool)
{
  if (!IsOwnedChannel(channel))
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  armed.fetch_and(~bit, std::memory_order_acq_rel);
  const uintptr_t ch = ChannelAddress(channel, 0u);

  // Ask for a graceful block-boundary stop first. Circular peripheral clients
  // keep their handshakes live while doing this so a trailing block can
  // retire. Do not issue CHABORT in the same cycle as CHEN disable: this DMAC
  // needs time to observe the graceful request.
  Register32(BASE + REG_CHEN) = bit << 8u;
  asm volatile("fence iorw, iorw" ::: "memory");
  if (!WaitForChannelDisabled(bit))
  {
    Register32(BASE + REG_CHABORT) = bit | (bit << 8u);
    asm volatile("fence iorw, iorw" ::: "memory");
    if (!WaitForChannelDisabled(bit))
    {
      // Keep ownership and the transfer record intact so a caller can retry
      // cancellation; an enabled hardware channel must never re-enter the
      // allocation pool.
      armed.fetch_or(bit, std::memory_order_release);
      return ErrorCode::TIMEOUT;
    }
  }
  Register32(ch + CH_INTCLEAR) = 0xFFFFFFFFu;
  active[channel] = {};
  return ErrorCode::OK;
}

int SG200XDMAC::InterruptHandler(int, void*)
{
  CheckInterrupt(true);
  return 0;
}

void SG200XDMAC::CheckInterrupt(bool in_isr)
{
  if (interrupt_dispatching.test_and_set(std::memory_order_acquire))
  {
    return;
  }
  // Order the MMIO status read with respect to descriptor writes and cache
  // maintenance performed by the DMA engine before dispatching callbacks.
  asm volatile("fence rw, rw" ::: "memory");
  for (Completion& completion : completed)
  {
    completion = {};
  }

  // A full-duplex client can stop its peer DMA channel from the first
  // completion callback. First snapshot and acknowledge every pending channel;
  // callbacks then observe stable completion records regardless of IRQ order.
  const uint32_t armed_channels = armed.load(std::memory_order_acquire);
  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    const uint32_t bit = 1u << channel;
    if ((armed_channels & bit) == 0u)
    {
      continue;
    }
    const uintptr_t ch = ChannelAddress(channel, 0u);
    const uint32_t status = Register32(ch + CH_INTSTATUS);
    if (status == 0u)
    {
      continue;
    }
    if (status != 0u)
    {
      Register32(ch + CH_INTCLEAR) = status;
    }
    Completion& completion = completed[channel];
    completion.transfer = active[channel];
    completion.status = status;
    completion.pending = true;
    const bool error = (status & INT_ERROR_MASK) != 0u;
    if (!error && completion.transfer.mode == Mode::NORMAL)
    {
      active[channel] = {};
      armed.fetch_and(~bit, std::memory_order_acq_rel);
    }
    if (error && completion.transfer.callback != nullptr)
    {
      // An error indication can leave the DesignWare channel active while the
      // peripheral handshake is no longer usable. Request its documented
      // immediate abort here; timeout/cancellation still use Abort()'s
      // graceful-stop attempt from task context.
      const uint32_t bit = 1u << channel;
      Register32(BASE + REG_CHABORT) = bit | (bit << 8u);
    }
  }

  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    const Completion& completion = completed[channel];
    const Transfer& transfer = completion.transfer;
    if (!completion.pending || transfer.callback == nullptr)
    {
      continue;
    }
    const ErrorCode result = (completion.status & INT_ERROR_MASK) == 0u
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
  interrupt_dispatching.clear(std::memory_order_release);
}

}  // namespace LibXR

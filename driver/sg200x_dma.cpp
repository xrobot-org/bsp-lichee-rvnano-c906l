#include "sg200x_dma.hpp"

#include "sg200x_mmio.hpp"

extern "C"
{
#include "arch_helpers.h"
  int request_irq(unsigned int irqn, int (*handler)(int, void*), unsigned long flags,
                  const char* name, void* argument) __attribute__((weak));
}

namespace LibXR
{
namespace
{
constexpr uintptr_t TOP_BASE = 0x03000000u;
constexpr uintptr_t CLOCK_GEN_BASE = 0x03002000u;
constexpr uintptr_t RESET_CTRL_BASE = 0x03003000u;
constexpr uintptr_t DMA_REMAP0 = TOP_BASE + 0x154u;
constexpr uintptr_t DMA_REMAP1 = TOP_BASE + 0x158u;
constexpr uintptr_t DMA_INT_MUX = TOP_BASE + 0x298u;
constexpr uint32_t CLOCK_ENABLE_1 = 0x04u;
constexpr uint32_t SOFT_RSTN_0 = 0x00u;
constexpr uint32_t CLOCK_SDMA_AXI = 1u << 1u;
// SOFT_RSTN_0 is active low; a one releases the SDMA controller from reset.
constexpr uint32_t RESET_SDMA = 1u << 18u;
constexpr uint32_t REG_CFG = 0x10u;
constexpr uint32_t REG_CHEN = 0x18u;
constexpr uint32_t REG_CHABORT = 0x28u;
constexpr uint32_t CHANNEL_BASE = 0x100u;
constexpr uint32_t CHANNEL_STRIDE = 0x100u;
constexpr uint32_t CH_SAR = 0x00u;
constexpr uint32_t CH_DAR = 0x08u;
constexpr uint32_t CH_BLOCK_TS = 0x10u;
constexpr uint32_t CH_CTL = 0x18u;
constexpr uint32_t CH_CFG = 0x20u;
constexpr uint32_t CH_LLP = 0x28u;
constexpr uint32_t CH_INTSTATUS_EN = 0x80u;
constexpr uint32_t CH_INTSTATUS = 0x88u;
// DW_axi_dmac channel interrupt registers are ordered enable, status,
// signal-enable, clear.  These offsets are also used by Linux's
// dw-axi-dmac driver on the SG200x.
constexpr uint32_t CH_INTSIGNAL_EN = 0x90u;
constexpr uint32_t CH_INTCLEAR = 0x98u;
constexpr uint64_t CTL_SMS = 1ull << 0;
constexpr uint64_t CTL_DMS = 1ull << 2;
constexpr uint64_t CTL_SINC = 1ull << 4;
constexpr uint64_t CTL_DINC = 1ull << 6;
constexpr uint32_t CTL_SRC_WIDTH_SHIFT = 8u;
constexpr uint32_t CTL_DST_WIDTH_SHIFT = 11u;
constexpr uint64_t CTL_IOC_BLOCK = 1ull << 58;
constexpr uint64_t CTL_LLI_LAST = 1ull << 62;
constexpr uint64_t CTL_LLI_VALID = 1ull << 63;
constexpr uint32_t CFG_TTFC_SHIFT = 32u;
constexpr uint32_t CFG_SRC_PER_SHIFT = 39u;
constexpr uint32_t CFG_DST_PER_SHIFT = 44u;
constexpr uint32_t CFG_PRIORITY_SHIFT = 49u;
constexpr uint32_t CFG_SRC_OSR_SHIFT = 55u;
constexpr uint32_t CFG_DST_OSR_SHIFT = 59u;
constexpr uint32_t CFG_SRC_MULTIBLK_LLI = 3u << 0u;
constexpr uint32_t CFG_DST_MULTIBLK_LLI = 3u << 2u;
constexpr uint32_t INT_BLOCK_DONE = 1u << 0;
// Bits 3 and 4 are normal transaction progress indications, not errors. Keep
// the error mask aligned with CHx_INTSTATUS: 5..14, 16..23, and 27..31.
constexpr uint32_t INT_ERROR_MASK = 0xF8FF7FE0u;
std::atomic<uint8_t> allocated{0u};
std::atomic<bool> initialized{false};
SG200XDMAC::Transfer active[SG200XDMAC::CHANNEL_COUNT]{};

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
}  // namespace

void SG200XDMAC::CleanForDevice(uintptr_t address, size_t size) noexcept
{
  if (size != 0u)
  {
    clean_dcache_range(address, size);
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
  bool expected = false;
  if (initialized.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
  {
    // The Linux clock framework does not guarantee that this clock remains
    // enabled while the remote C906L owns a transfer.  Preserve every other
    // CRG setting, enable only SDMA's AXI clock, then release its active-low
    // reset before touching the controller registers.  SG200x TRM 11.4.1
    // makes both settings a prerequisite for DMAC access.
    Register32(CLOCK_GEN_BASE + CLOCK_ENABLE_1) |= CLOCK_SDMA_AXI;
    Register32(RESET_CTRL_BASE + SOFT_RSTN_0) |= RESET_SDMA;
    // DMAC_EN and global interrupt generation.
    Register32(BASE + REG_CFG) = 3u;
    // Route all eight channel interrupts to the C906L CPU2 domain. The TRM
    // exposes three 9-bit CPU masks at [8:0], [18:10], and [28:20]; the C906L
    // FreeRTOS interrupt map identifies this core as CPU2.
    Register32(DMA_INT_MUX) =
        (Register32(DMA_INT_MUX) & ~(0x1FFu << 20u)) | (0x1FFu << 20u);
    if (request_irq == nullptr ||
        request_irq(IRQ, &InterruptHandler, 0u, "sg200x-dma", nullptr) != 0)
    {
      initialized.store(false, std::memory_order_release);
      return ErrorCode::NOT_SUPPORT;
    }
  }
  return ErrorCode::OK;
}

ErrorCode SG200XDMAC::Acquire(uint8_t& channel)
{
  const ErrorCode initialized_result = Initialize();
  if (initialized_result != ErrorCode::OK)
  {
    return initialized_result;
  }
  uint8_t used = allocated.load(std::memory_order_acquire);
  for (;;)
  {
    for (uint8_t index = 0u; index < CHANNEL_COUNT; ++index)
    {
      const uint8_t bit = static_cast<uint8_t>(1u << index);
      if ((used & bit) == 0u)
      {
        const uint8_t desired = static_cast<uint8_t>(used | bit);
        if (allocated.compare_exchange_weak(used, desired, std::memory_order_acq_rel,
                                            std::memory_order_acquire))
        {
          channel = index;
          return ErrorCode::OK;
        }
        break;
      }
    }
    if (used == 0xFFu)
    {
      return ErrorCode::BUSY;
    }
  }
}

void SG200XDMAC::Release(uint8_t channel)
{
  if (channel >= CHANNEL_COUNT)
  {
    return;
  }
  (void)Abort(channel);
  allocated.fetch_and(static_cast<uint8_t>(~(1u << channel)), std::memory_order_release);
}

ErrorCode SG200XDMAC::Start(uint8_t channel, const Transfer& transfer)
{
  if (channel >= CHANNEL_COUNT || transfer.memory == 0u || transfer.peripheral == 0u ||
      transfer.count == 0u || transfer.count > 0x400000u || transfer.callback == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  const uint8_t bit = static_cast<uint8_t>(1u << channel);
  if ((allocated.load(std::memory_order_acquire) & bit) == 0u)
  {
    return ErrorCode::STATE_ERR;
  }

  const size_t bytes = transfer.count << static_cast<uint8_t>(transfer.width);
  if (transfer.direction == Direction::MEMORY_TO_PERIPHERAL ||
      transfer.direction == Direction::MEMORY_TO_MEMORY)
  {
    CleanForDevice(transfer.memory, bytes);
  }
  else
  {
    InvalidateForCpu(transfer.memory, bytes);
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
  // The SDK uses master 2 for both sides of its I2C DMA LLI. This is the
  // memory/peripheral AXI path exposed to the C906L on SG200x.
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
  control |= CTL_LLI_LAST | CTL_LLI_VALID;
  descriptor.control_low = static_cast<uint32_t>(control);
  descriptor.control_high = static_cast<uint32_t>(control >> 32u);
  // TOP remap selects the peripheral request. SRC_PER/DST_PER select the
  // physical DMA handshake line. Keep the linked-list type bits for external
  // peripherals too: replacing cfg here would leave CH_LLP programmed but
  // make the controller treat the transfer as a non-LLI transaction.
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
  Register32(ch + CH_LLP) =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&descriptor));
  Register32(ch + CH_LLP + 4u) =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&descriptor) >> 32u);
  Register32(ch + CH_INTSTATUS_EN) = INT_BLOCK_DONE | INT_ERROR_MASK;
  Register32(ch + CH_INTSIGNAL_EN) = INT_BLOCK_DONE | INT_ERROR_MASK;
  active[channel] = transfer;
  Register32(BASE + REG_CHEN) =
      static_cast<uint32_t>(bit) | (static_cast<uint32_t>(bit) << 8u);
  return ErrorCode::OK;
}

ErrorCode SG200XDMAC::Abort(uint8_t channel)
{
  if (channel >= CHANNEL_COUNT)
  {
    return ErrorCode::ARG_ERR;
  }
  const uint32_t bit = 1u << channel;
  // Ask for a graceful stop first.  A dead peripheral handshake can prevent
  // that from completing, in which case the TRM's explicit abort request is
  // the documented recovery mechanism.
  Register32(BASE + REG_CHEN) = bit << 8u;
  for (uint32_t attempt = 0u; attempt < 10000u; ++attempt)
  {
    if ((Register32(BASE + REG_CHEN) & bit) == 0u)
    {
      break;
    }
  }
  if ((Register32(BASE + REG_CHEN) & bit) != 0u)
  {
    Register32(BASE + REG_CHABORT) = bit | (bit << 8u);
  }
  Register32(ChannelAddress(channel, CH_INTCLEAR)) = 0xFFFFFFFFu;
  active[channel] = {};
  return ErrorCode::OK;
}

void SG200XDMAC::CheckInterrupt()
{
  for (uint8_t channel = 0u; channel < CHANNEL_COUNT; ++channel)
  {
    const uintptr_t ch = ChannelAddress(channel, 0u);
    const uint32_t status = Register32(ch + CH_INTSTATUS);
    if (status == 0u)
    {
      continue;
    }
    Register32(ch + CH_INTCLEAR) = status;
    const Transfer transfer = active[channel];
    active[channel] = {};
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
    if (transfer.callback != nullptr)
    {
      transfer.callback(transfer.context, (status & INT_ERROR_MASK) == 0u
                                              ? ErrorCode::OK
                                              : ErrorCode::FAILED);
    }
  }
}

int SG200XDMAC::InterruptHandler(int, void*)
{
  CheckInterrupt();
  return 0;
}

}  // namespace LibXR

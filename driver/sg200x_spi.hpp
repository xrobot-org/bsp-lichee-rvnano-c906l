#pragma once

#include <atomic>
#include <cstdint>

#include "sg200x_dma.hpp"
#include "sg200x_ll_spi.h"
#include "sg200x_rcc.hpp"
#include "sg200x_ll.h"
#include "spi.hpp"

namespace LibXR
{

/**
 * @brief DMA-only DesignWare SSI master for the SG200x C906L.
 *
 * The TRM specifies an 8-entry RX/TX FIFO, 8-bit Motorola SPI frames, and
 * an even BAUDR divider from 2 through 65534. The instance-owned RX/TX
 * buffers are used as DMA staging storage, matching STM32SPI: ordinary APIs
 * accept arbitrary caller buffers while Transfer() submits the active
 * internal buffers directly. Transfers are intentionally rejected from ISR
 * context because DMA ownership is established by task code.
 */
class SG200XSPI final : public SPI
{
 public:
  using Controller = SG200XRCC::SpiController;

  struct DmaChannels
  {
    uint8_t rx = 4u;
    uint8_t tx = 5u;
  };

  SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
            RawData tx_buffer, Configuration config = {},
            DmaChannels dma_channels = {4u, 5u});

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr = false) override;
  ErrorCode Read(RawData read_data, OperationRW& op, bool in_isr = false) override;
  ErrorCode Write(ConstRawData write_data, OperationRW& op, bool in_isr = false) override;
  ErrorCode SetConfig(Configuration config) override;
  uint32_t GetMaxBusSpeed() const override;
  Prescaler GetMaxPrescaler() const override;
  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) override;
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr = false) override;
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr = false) override;

  /** Start a zero-copy full-duplex DMA ring on the active internal buffers. */
  ErrorCode StartCircularTransfer(size_t size, OperationRW& op, bool in_isr = false);
  /** Stop a circular transfer without emitting an additional completion. */
  ErrorCode StopCircularTransfer(bool in_isr = false);

  [[nodiscard]] bool IsValid() const noexcept { return regs_ != nullptr; }
  [[nodiscard]] uint32_t ActualBusSpeed() const noexcept;
  [[nodiscard]] uint32_t DmaReceiveCount() const noexcept
  {
    return dma_receive_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] bool IsCircularTransferActive() const noexcept
  {
    return circular_active_.load(std::memory_order_acquire);
  }

 private:
  static constexpr uint32_t IDLE_WAIT_ATTEMPTS = 100000u;

  enum class DmaBufferMode : uint8_t
  {
    STAGED,
    INTERNAL,
  };

  bool TryLock() noexcept;
  void Unlock() noexcept;
  ErrorCode StartDmaTransfer(RawData read_data, ConstRawData write_data, OperationRW& op,
                             DmaBufferMode buffer_mode = DmaBufferMode::STAGED,
                             bool has_prefix = false, uint8_t prefix = 0u,
                             sg200x_ll_dma_mode_t dma_mode = LL_DMA_MODE_NORMAL);
  void OnDmaComplete(bool rx, ErrorCode result, bool in_isr);
  void FinishDma(ErrorCode result, bool in_isr = false);
  void CancelDmaTransfer();
  static void DmaTxCallback(void* context, ErrorCode result, bool in_isr);
  static void DmaRxCallback(void* context, ErrorCode result, bool in_isr);
  ErrorCode Disable() const;
  ErrorCode WaitForIdle() const;
  ErrorCode CheckError() const;

  spi_t* regs_ = nullptr;
  uint8_t controller_index_ = 0u;
  uint32_t input_clock_hz_ = 0u;
  uint32_t baud_divider_ = 0u;
  uint32_t chip_select_mask_ = 0u;
  std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
  std::atomic<bool> finishing_{false};
  std::atomic<bool> circular_active_{false};
  DmaChannels dma_channels_{};
  uint8_t tx_dma_channel_ = 0xFFu;
  uint8_t rx_dma_channel_ = 0xFFu;
  std::atomic<uint32_t> dma_receive_count_{0u};
  OperationRW active_op_{};
  AsyncBlockWait block_wait_{};
  RawData active_read_{};
  size_t active_read_offset_ = 0u;
  bool active_read_needs_copy_ = false;
  size_t active_dma_rx_bytes_ = 0u;
};

}  // namespace LibXR

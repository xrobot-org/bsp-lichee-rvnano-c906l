#pragma once

#include <atomic>
#include <cstdint>

#include "sg200x_dma.hpp"
#include "sg200x_rcc.hpp"
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

  static constexpr uintptr_t SPI0_BASE = 0x04180000u;
  SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
            RawData tx_buffer, Configuration config = {});

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
  ErrorCode StartCircularTransfer(size_t size, OperationRW& op,
                                  bool in_isr = false);
  /** Stop a circular transfer without emitting an additional completion. */
  ErrorCode StopCircularTransfer(bool in_isr = false);

  [[nodiscard]] bool IsValid() const noexcept { return base_ != 0u; }
  [[nodiscard]] uint32_t ActualBusSpeed() const noexcept;
  [[nodiscard]] bool IsCircularTransferActive() const noexcept
  {
    return circular_active_.load(std::memory_order_acquire);
  }

 private:
  static constexpr uintptr_t CONTROLLER_STRIDE = 0x10000u;
  static constexpr uint8_t CONTROLLER_COUNT = 4u;
  static constexpr uint32_t REG_CTRLR0 = 0x00u;
  static constexpr uint32_t REG_SPIENR = 0x08u;
  static constexpr uint32_t REG_SER = 0x10u;
  static constexpr uint32_t REG_BAUDR = 0x14u;
  static constexpr uint32_t REG_TXFTLR = 0x18u;
  static constexpr uint32_t REG_RXFTLR = 0x1Cu;
  static constexpr uint32_t REG_SR = 0x28u;
  static constexpr uint32_t REG_IMR = 0x2Cu;
  static constexpr uint32_t REG_RISR = 0x34u;
  static constexpr uint32_t REG_ICR = 0x48u;
  static constexpr uint32_t REG_DMACR = 0x4Cu;
  static constexpr uint32_t REG_DMATDLR = 0x50u;
  static constexpr uint32_t REG_DMARDLR = 0x54u;
  static constexpr uint32_t REG_DR = 0x60u;
  static constexpr uint32_t CTRLR0_SRL = 1u << 11u;
  static constexpr uint32_t CTRLR0_TMOD_TXRX = 0u << 8u;
  static constexpr uint32_t CTRLR0_SCPOL = 1u << 7u;
  static constexpr uint32_t CTRLR0_SCPH = 1u << 6u;
  static constexpr uint32_t CTRLR0_MOTOROLA = 0u << 4u;
  static constexpr uint32_t CTRLR0_DFS_8BIT = 7u;
  static constexpr uint32_t SR_BUSY = 1u << 0u;
  static constexpr uint32_t RISR_ERROR_MASK =
      (1u << 1u) | (1u << 2u) | (1u << 3u) | (1u << 5u);
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
                             SG200XDMAC::Mode dma_mode = SG200XDMAC::Mode::NORMAL);
  void OnDmaComplete(bool rx, ErrorCode result, bool in_isr);
  void FinishDma(ErrorCode result, bool in_isr = false);
  void CancelDmaTransfer();
  static void DmaTxCallback(void* context, ErrorCode result, bool in_isr);
  static void DmaRxCallback(void* context, ErrorCode result, bool in_isr);
  ErrorCode Disable() const;
  ErrorCode WaitForIdle() const;
  ErrorCode CheckError() const;

  uintptr_t base_ = 0u;
  uint32_t input_clock_hz_ = 0u;
  uint32_t baud_divider_ = 0u;
  uint32_t chip_select_mask_ = 0u;
  std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
  std::atomic<bool> finishing_{false};
  std::atomic<bool> circular_active_{false};
  uint8_t tx_dma_channel_ = 0xFFu;
  uint8_t rx_dma_channel_ = 0xFFu;
  OperationRW active_op_{};
  AsyncBlockWait block_wait_{};
  RawData active_read_{};
  size_t active_read_offset_ = 0u;
  bool active_read_needs_copy_ = false;
};

}  // namespace LibXR

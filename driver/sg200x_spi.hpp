#pragma once

#include <atomic>
#include <cstdint>

#include "sg200x_dma.hpp"
#include "spi.hpp"

namespace LibXR
{

/**
 * @brief DMA-only DesignWare SSI master for the SG200x C906L.
 *
 * The TRM specifies an 8-entry RX/TX FIFO, 8-bit Motorola SPI frames, and
 * an even BAUDR divider from 2 through 65534. Transfers are intentionally
 * rejected from ISR context because DMA ownership is established by task code.
 */
class SG200XSPI final : public SPI
{
 public:
  enum class Controller : uint8_t
  {
    SPI0 = 0u,
    SPI1,
    SPI2,
    SPI3,
  };

  static constexpr uintptr_t SPI0_BASE = 0x04180000u;
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 187500000u;

  SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
            RawData tx_buffer, uint32_t input_clock_hz = DEFAULT_CLOCK_HZ,
            Configuration config = {});

  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW& op,
                         bool in_isr = false) override;
  ErrorCode SetConfig(Configuration config) override;
  uint32_t GetMaxBusSpeed() const override;
  Prescaler GetMaxPrescaler() const override;
  ErrorCode Transfer(size_t size, OperationRW& op, bool in_isr = false) override;
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                     bool in_isr = false) override;
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                    bool in_isr = false) override;

  [[nodiscard]] bool IsValid() const noexcept { return base_ != 0u; }
  [[nodiscard]] uint32_t ActualBusSpeed() const noexcept;

 private:
  static constexpr uintptr_t CONTROLLER_STRIDE = 0x10000u;
  static constexpr uint8_t CONTROLLER_COUNT = 4u;

  static constexpr uint32_t REG_CTRLR0 = 0x00u;
  static constexpr uint32_t REG_SPIENR = 0x08u;
  static constexpr uint32_t REG_SER = 0x10u;
  static constexpr uint32_t REG_BAUDR = 0x14u;
  static constexpr uint32_t REG_TXFTLR = 0x18u;
  static constexpr uint32_t REG_RXFTLR = 0x1Cu;
  static constexpr uint32_t REG_TXFLR = 0x20u;
  static constexpr uint32_t REG_RXFLR = 0x24u;
  static constexpr uint32_t REG_SR = 0x28u;
  static constexpr uint32_t REG_IMR = 0x2Cu;
  static constexpr uint32_t REG_RISR = 0x34u;
  static constexpr uint32_t REG_ICR = 0x48u;
  static constexpr uint32_t REG_DMACR = 0x4Cu;
  static constexpr uint32_t REG_DMATDLR = 0x50u;
  static constexpr uint32_t REG_DMARDLR = 0x54u;
  static constexpr uint32_t REG_DR = 0x60u;

  static constexpr uint32_t CTRLR0_TMOD_TXRX = 0u << 8;
  static constexpr uint32_t CTRLR0_SCPOL = 1u << 7;
  static constexpr uint32_t CTRLR0_SCPH = 1u << 6;
  static constexpr uint32_t CTRLR0_MOTOROLA = 0u << 4;
  static constexpr uint32_t CTRLR0_DFS_8BIT = 7u;
  static constexpr uint32_t SR_BUSY = 1u << 0;
  static constexpr uint32_t SR_TFNF = 1u << 1;
  static constexpr uint32_t SR_TFE = 1u << 2;
  static constexpr uint32_t SR_RFNE = 1u << 3;
  static constexpr uint32_t RISR_ERROR_MASK =
      (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5);

  bool TryLock() noexcept;
  void Unlock() noexcept;
  ErrorCode StartDmaTransfer(RawData read_data, ConstRawData write_data, OperationRW& op,
                             size_t read_offset = 0u);
  void OnDmaComplete(bool rx, ErrorCode result);
  void FinishDma(ErrorCode result);
  static void DmaTxCallback(void* context, ErrorCode result);
  static void DmaRxCallback(void* context, ErrorCode result);
  ErrorCode Disable() const;
  ErrorCode CheckError() const;

  template <typename OperationType>
  static ErrorCode Complete(OperationType& op, bool in_isr, ErrorCode result)
  {
    if (op.type != OperationType::OperationType::BLOCK)
    {
      op.UpdateStatus(in_isr, result);
    }
    return result;
  }

  uintptr_t base_ = 0u;
  uint32_t input_clock_hz_ = 0u;
  uint32_t baud_divider_ = 0u;
  uint32_t chip_select_mask_ = 0u;
  std::atomic_flag busy_ = ATOMIC_FLAG_INIT;
  uint8_t tx_dma_channel_ = 0xFFu;
  uint8_t rx_dma_channel_ = 0xFFu;
  bool tx_done_ = false;
  bool rx_done_ = false;
  OperationRW active_op_{};
  AsyncBlockWait block_wait_{};
  RawData active_read_{};
  size_t active_frames_ = 0u;
  size_t active_read_offset_ = 0u;
};

}  // namespace LibXR

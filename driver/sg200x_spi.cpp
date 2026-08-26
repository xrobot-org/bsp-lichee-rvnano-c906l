#include "sg200x_spi.hpp"

#include "sg200x_mmio.hpp"

namespace LibXR
{

SG200XSPI::SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
                     RawData tx_buffer, uint32_t input_clock_hz, Configuration config)
    : SPI(rx_buffer, tx_buffer), input_clock_hz_(input_clock_hz)
{
  const uint8_t index = static_cast<uint8_t>(controller);
  if (index >= CONTROLLER_COUNT || chip_select >= 32u || input_clock_hz_ == 0u)
  {
    return;
  }
  base_ = SPI0_BASE + static_cast<uintptr_t>(index) * CONTROLLER_STRIDE;
  chip_select_mask_ = static_cast<uint32_t>(1u) << chip_select;
  if (config.prescaler == Prescaler::UNKNOWN)
  {
    config.prescaler = Prescaler::DIV_2;
  }
  if (SetConfig(config) != ErrorCode::OK)
  {
    base_ = 0u;
  }
}

bool SG200XSPI::TryLock() noexcept
{
  return !busy_.test_and_set(std::memory_order_acquire);
}

void SG200XSPI::Unlock() noexcept { busy_.clear(std::memory_order_release); }

ErrorCode SG200XSPI::Disable() const
{
  Register32(base_, REG_SER) = 0u;
  Register32(base_, REG_SPIENR) = 0u;
  return ErrorCode::OK;
}

ErrorCode SG200XSPI::CheckError() const
{
  if ((Register32(base_, REG_RISR) & RISR_ERROR_MASK) != 0u)
  {
    const uint32_t clear = Register32(base_, REG_ICR);
    (void)clear;
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode SG200XSPI::SetConfig(Configuration config)
{
  if (!IsValid() || config.prescaler == Prescaler::UNKNOWN || !TryLock())
  {
    return IsValid() ? ErrorCode::NOT_SUPPORT : ErrorCode::ARG_ERR;
  }

  const uint32_t requested_divider = PrescalerToDiv(config.prescaler);
  if (requested_divider == 0u)
  {
    Unlock();
    return ErrorCode::ARG_ERR;
  }

  // LibXR's maximum rate is the SSI's real maximum (ssi_clk / 2).  Each
  // LibXR prescaler therefore maps to twice its numeric hardware divider.
  const uint64_t divider64 = static_cast<uint64_t>(requested_divider) * 2u;
  if (divider64 > 65534u)
  {
    Unlock();
    return ErrorCode::NOT_SUPPORT;
  }
  const uint32_t divider = static_cast<uint32_t>(divider64);

  (void)Disable();
  uint32_t ctrlr0 = CTRLR0_TMOD_TXRX | CTRLR0_MOTOROLA | CTRLR0_DFS_8BIT;
  if (config.clock_polarity == ClockPolarity::HIGH)
  {
    ctrlr0 |= CTRLR0_SCPOL;
  }
  if (config.clock_phase == ClockPhase::EDGE_2)
  {
    ctrlr0 |= CTRLR0_SCPH;
  }
  Register32(base_, REG_CTRLR0) = ctrlr0;
  Register32(base_, REG_BAUDR) = divider;
  Register32(base_, REG_TXFTLR) = 0u;
  Register32(base_, REG_RXFTLR) = 0u;
  Register32(base_, REG_IMR) = 0u;
  Register32(base_, REG_DMACR) = 0u;
  const uint32_t clear = Register32(base_, REG_ICR);
  (void)clear;
  baud_divider_ = divider;
  GetConfig() = config;
  Unlock();
  return ErrorCode::OK;
}

uint32_t SG200XSPI::GetMaxBusSpeed() const { return input_clock_hz_ / 2u; }

SG200XSPI::Prescaler SG200XSPI::GetMaxPrescaler() const { return Prescaler::DIV_16384; }

uint32_t SG200XSPI::ActualBusSpeed() const noexcept
{
  return baud_divider_ == 0u ? 0u : input_clock_hz_ / baud_divider_;
}

ErrorCode SG200XSPI::StartDmaTransfer(RawData read_data, ConstRawData write_data,
                                      OperationRW& op, size_t read_offset)
{
  if ((read_data.size_ != 0u && read_data.addr_ == nullptr) ||
      (write_data.size_ != 0u && write_data.addr_ == nullptr) || !TryLock())
  {
    return ErrorCode::BUSY;
  }
  const size_t frames =
      read_data.size_ > write_data.size_ ? read_data.size_ : write_data.size_;
  if (frames == 0u)
  {
    Unlock();
    return Complete(op, false, ErrorCode::OK);
  }
  RawData dma_rx = GetRxBuffer();
  RawData dma_tx = GetTxBuffer();
  if (dma_rx.addr_ == nullptr || dma_tx.addr_ == nullptr || frames > dma_rx.size_ ||
      frames > dma_tx.size_ || read_offset > frames ||
      read_data.size_ + read_offset > frames)
  {
    Unlock();
    return ErrorCode::SIZE_ERR;
  }
  auto* tx = static_cast<uint8_t*>(dma_tx.addr_);
  const auto* input = static_cast<const uint8_t*>(write_data.addr_);
  for (size_t i = 0u; i < frames; ++i)
  {
    tx[i] = i < write_data.size_ ? input[i] : 0xFFu;
  }

  ErrorCode result = SG200XDMAC::Acquire(rx_dma_channel_);
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Acquire(tx_dma_channel_);
  }
  if (result != ErrorCode::OK)
  {
    if (rx_dma_channel_ != 0xFFu)
    {
      SG200XDMAC::Release(rx_dma_channel_);
    }
    rx_dma_channel_ = tx_dma_channel_ = 0xFFu;
    Unlock();
    return result;
  }

  active_op_ = op;
  active_read_ = read_data;
  active_frames_ = frames;
  active_read_offset_ = read_offset;
  tx_done_ = false;
  rx_done_ = false;
  if (op.type == OperationRW::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }

  (void)Disable();
  // With the documented zero watermarks, TX DMA requests as soon as the FIFO
  // is empty and RX DMA requests for every received frame. This avoids a
  // CPU-side FIFO service path for short transfers as well as long ones.
  Register32(base_, REG_DMATDLR) = 0u;
  Register32(base_, REG_DMARDLR) = 0u;
  Register32(base_, REG_DMACR) = 3u;
  Register32(base_, REG_SPIENR) = 1u;
  Register32(base_, REG_SER) = chip_select_mask_;
  const SG200XDMAC::Transfer rx_transfer{
      .memory = reinterpret_cast<uintptr_t>(dma_rx.addr_),
      .peripheral = base_ + REG_DR,
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          static_cast<uint8_t>(SG200XDMAC::Request::SPI0_RX) +
          static_cast<uint8_t>((base_ - SPI0_BASE) / CONTROLLER_STRIDE) * 2u),
      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
      .width = SG200XDMAC::Width::BYTE,
      .callback = &DmaRxCallback,
      .context = this};
  const SG200XDMAC::Transfer tx_transfer{
      .memory = reinterpret_cast<uintptr_t>(dma_tx.addr_),
      .peripheral = base_ + REG_DR,
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          static_cast<uint8_t>(SG200XDMAC::Request::SPI0_TX) +
          static_cast<uint8_t>((base_ - SPI0_BASE) / CONTROLLER_STRIDE) * 2u),
      .direction = SG200XDMAC::Direction::MEMORY_TO_PERIPHERAL,
      .width = SG200XDMAC::Width::BYTE,
      .callback = &DmaTxCallback,
      .context = this};
  result = SG200XDMAC::Start(rx_dma_channel_, rx_transfer);
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Start(tx_dma_channel_, tx_transfer);
  }
  if (result != ErrorCode::OK)
  {
    FinishDma(result);
    return result;
  }
  op.MarkAsRunning();
  if (op.type != OperationRW::OperationType::BLOCK)
  {
    return ErrorCode::OK;
  }
  const ErrorCode wait_result = block_wait_.Wait(op.data.sem_info.timeout);
  if (wait_result == ErrorCode::TIMEOUT)
  {
    FinishDma(ErrorCode::TIMEOUT);
  }
  return wait_result;
}

void SG200XSPI::DmaTxCallback(void* context, ErrorCode result)
{
  static_cast<SG200XSPI*>(context)->OnDmaComplete(false, result);
}

void SG200XSPI::DmaRxCallback(void* context, ErrorCode result)
{
  static_cast<SG200XSPI*>(context)->OnDmaComplete(true, result);
}

void SG200XSPI::OnDmaComplete(bool rx, ErrorCode result)
{
  if (result != ErrorCode::OK)
  {
    FinishDma(result);
    return;
  }
  if (rx)
  {
    rx_done_ = true;
  }
  else
  {
    tx_done_ = true;
  }
  if (rx_done_ && tx_done_)
  {
    FinishDma(ErrorCode::OK);
  }
}

void SG200XSPI::FinishDma(ErrorCode result)
{
  if (!busy_.test(std::memory_order_acquire))
  {
    return;
  }
  if (result == ErrorCode::OK)
  {
    // DMA can finish normally after an SSI FIFO or multi-master error. Read
    // RISR before disabling the peripheral so the transfer is not reported as
    // successful merely because both DMA descriptors reached completion.
    result = CheckError();
  }
  Register32(base_, REG_DMACR) = 0u;
  (void)Disable();
  if (rx_dma_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(rx_dma_channel_);
  }
  if (tx_dma_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(tx_dma_channel_);
  }
  rx_dma_channel_ = tx_dma_channel_ = 0xFFu;
  if (result == ErrorCode::OK && active_read_.size_ != 0u)
  {
    const auto* source = static_cast<const uint8_t*>(GetRxBuffer().addr_);
    auto* destination = static_cast<uint8_t*>(active_read_.addr_);
    for (size_t i = 0u; i < active_read_.size_; ++i)
    {
      destination[i] = source[i + active_read_offset_];
    }
  }
  if (result == ErrorCode::OK)
  {
    SwitchBuffer();
  }
  if (active_op_.type == OperationRW::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(true, result);
  }
  else
  {
    active_op_.UpdateStatus(true, result);
  }
  active_op_ = {};
  active_read_ = {};
  Unlock();
}

ErrorCode SG200XSPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                  OperationRW& op, bool in_isr)
{
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return StartDmaTransfer(read_data, write_data, op);
}

ErrorCode SG200XSPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  if (size > rx.size_ || size > tx.size_)
  {
    return ErrorCode::SIZE_ERR;
  }
  return ReadAndWrite({rx.addr_, size}, {tx.addr_, size}, op, in_isr);
}

ErrorCode SG200XSPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                              bool in_isr)
{
  RawData tx = GetTxBuffer();
  if (reg > 0xFFu || tx.addr_ == nullptr || write_data.size_ + 1u > tx.size_)
  {
    return ErrorCode::SIZE_ERR;
  }
  auto* bytes = static_cast<uint8_t*>(tx.addr_);
  bytes[0] = static_cast<uint8_t>(reg & 0x7Fu);
  const auto* input = static_cast<const uint8_t*>(write_data.addr_);
  for (size_t i = 0u; i < write_data.size_; ++i)
  {
    bytes[i + 1u] = input[i];
  }
  return ReadAndWrite({}, {bytes, write_data.size_ + 1u}, op, in_isr);
}

ErrorCode SG200XSPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                             bool in_isr)
{
  RawData tx = GetTxBuffer();
  RawData rx = GetRxBuffer();
  if (reg > 0xFFu || tx.addr_ == nullptr || rx.addr_ == nullptr ||
      read_data.size_ + 1u > tx.size_ || read_data.size_ + 1u > rx.size_)
  {
    return ErrorCode::SIZE_ERR;
  }
  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  tx_bytes[0] = static_cast<uint8_t>(reg | 0x80u);
  for (size_t i = 0u; i < read_data.size_; ++i)
  {
    tx_bytes[i + 1u] = 0xFFu;
  }
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return StartDmaTransfer(read_data, {tx.addr_, read_data.size_ + 1u}, op, 1u);
}

}  // namespace LibXR

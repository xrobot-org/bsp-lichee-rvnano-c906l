#include "sg200x_spi.hpp"

#include "sg200x_mmio.hpp"

namespace LibXR
{

SG200XSPI::SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
                     RawData tx_buffer, Configuration config)
    : SPI(rx_buffer, tx_buffer)
{
  const uint8_t index = static_cast<uint8_t>(controller);
  if (index >= CONTROLLER_COUNT || chip_select >= 32u || rx_buffer.addr_ == nullptr ||
      tx_buffer.addr_ == nullptr || rx_buffer.size_ == 0u || tx_buffer.size_ == 0u ||
      (reinterpret_cast<uintptr_t>(rx_buffer.addr_) % HW_CACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(tx_buffer.addr_) % HW_CACHE_LINE_SIZE) != 0u)
  {
    return;
  }
  const auto peripheral = static_cast<SG200XRCC::PeripheralId>(
      static_cast<uint8_t>(SG200XRCC::PeripheralId::Spi0) + index);
  SG200XRCC& rcc = SG200XRCC::Instance();
  if (rcc.PreparePeripheral(peripheral) != ErrorCode::OK ||
      (input_clock_hz_ = rcc.ClockRate(SG200XRCC::ClockId::Spi)) == 0u)
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

ErrorCode SG200XSPI::WaitForIdle() const
{
  for (uint32_t attempt = 0u; attempt < IDLE_WAIT_ATTEMPTS; ++attempt)
  {
    if ((Register32(base_, REG_SR) & SR_BUSY) == 0u)
    {
      return ErrorCode::OK;
    }
  }
  return ErrorCode::TIMEOUT;
}

ErrorCode SG200XSPI::SetConfig(Configuration config)
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  if (config.prescaler == Prescaler::UNKNOWN ||
      (config.clock_polarity != ClockPolarity::LOW &&
       config.clock_polarity != ClockPolarity::HIGH) ||
      (config.clock_phase != ClockPhase::EDGE_1 && config.clock_phase != ClockPhase::EDGE_2))
  {
    return ErrorCode::ARG_ERR;
  }
  if (!TryLock())
  {
    return ErrorCode::BUSY;
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
#ifdef SG200X_SPI_INTERNAL_LOOPBACK
  // SG2002 TRM CTRLR0[11]: route the TX shift-register output directly to RX.
  // This controller-level test mode intentionally bypasses the package pads.
  ctrlr0 |= CTRLR0_SRL;
#endif
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
                                      OperationRW& op, DmaBufferMode buffer_mode,
                                      bool has_prefix, uint8_t prefix,
                                      SG200XDMAC::Mode dma_mode)
{
  if ((read_data.size_ != 0u && read_data.addr_ == nullptr) ||
      (write_data.size_ != 0u && write_data.addr_ == nullptr))
  {
    return ErrorCode::ARG_ERR;
  }
  if (!TryLock())
  {
    return ErrorCode::BUSY;
  }
  // A late DMA callback can only clear the previous transfer while FinishDma
  // still owns its cleanup. Do not let a new caller reuse the state in that
  // short handoff window.
  if (finishing_.load(std::memory_order_acquire))
  {
    Unlock();
    return ErrorCode::BUSY;
  }
  const size_t payload_frames =
      read_data.size_ > write_data.size_ ? read_data.size_ : write_data.size_;
  if (payload_frames == 0u && !has_prefix)
  {
    Unlock();
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }
  if (has_prefix && payload_frames == static_cast<size_t>(-1))
  {
    Unlock();
    return ErrorCode::SIZE_ERR;
  }
  const size_t read_offset = has_prefix ? 1u : 0u;
  const size_t frames = payload_frames + read_offset;

  RawData dma_rx = GetRxBuffer();
  RawData dma_tx = GetTxBuffer();
  if (buffer_mode == DmaBufferMode::INTERNAL)
  {
    dma_rx = read_data;
    dma_tx = {const_cast<void*>(write_data.addr_), write_data.size_};
  }
  if (dma_rx.addr_ == nullptr || dma_tx.addr_ == nullptr ||
      (reinterpret_cast<uintptr_t>(dma_rx.addr_) % HW_CACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(dma_tx.addr_) % HW_CACHE_LINE_SIZE) != 0u ||
      frames > dma_rx.size_ || frames > dma_tx.size_ || read_offset > frames ||
      read_data.size_ > frames - read_offset)
  {
    Unlock();
    return ErrorCode::SIZE_ERR;
  }
  if (buffer_mode == DmaBufferMode::INTERNAL &&
      (has_prefix || read_data.size_ != frames || write_data.size_ != frames))
  {
    Unlock();
    return ErrorCode::ARG_ERR;
  }
  if (buffer_mode == DmaBufferMode::STAGED)
  {
    auto* destination = static_cast<uint8_t*>(dma_tx.addr_);
    const auto* source = static_cast<const uint8_t*>(write_data.addr_);
    if (has_prefix)
    {
      destination[0] = prefix;
    }
    for (size_t index = 0u; index < payload_frames; ++index)
    {
      destination[index + read_offset] =
          index < write_data.size_ ? source[index] : 0xFFu;
    }
  }

  const uintptr_t rx_start = reinterpret_cast<uintptr_t>(dma_rx.addr_);
  const uintptr_t tx_start = reinterpret_cast<uintptr_t>(dma_tx.addr_);

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
  active_read_offset_ = read_offset;
  active_read_needs_copy_ = buffer_mode == DmaBufferMode::STAGED;
  circular_active_.store(dma_mode == SG200XDMAC::Mode::CIRCULAR,
                         std::memory_order_release);
  if (op.type == OperationRW::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  // A DMA interrupt is allowed to preempt immediately after its channel is
  // armed. Publish RUNNING before that point so a fast transfer cannot leave
  // a POLLING operation in RUNNING after its completion has set it to DONE.
  op.MarkAsRunning();

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
      .memory = rx_start,
      .peripheral = base_ + REG_DR,
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          static_cast<uint8_t>(SG200XDMAC::Request::SPI0_RX) +
          static_cast<uint8_t>((base_ - SPI0_BASE) / CONTROLLER_STRIDE) * 2u),
      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
      .width = SG200XDMAC::Width::BYTE,
      .mode = dma_mode,
      .callback = &DmaRxCallback,
      .context = this};
  const SG200XDMAC::Transfer tx_transfer{
      .memory = tx_start,
      .peripheral = base_ + REG_DR,
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          static_cast<uint8_t>(SG200XDMAC::Request::SPI0_TX) +
          static_cast<uint8_t>((base_ - SPI0_BASE) / CONTROLLER_STRIDE) * 2u),
      .direction = SG200XDMAC::Direction::MEMORY_TO_PERIPHERAL,
      .width = SG200XDMAC::Width::BYTE,
      .mode = dma_mode,
      .callback = &DmaTxCallback,
      .context = this};
  result = SG200XDMAC::Start(rx_dma_channel_, rx_transfer);
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Start(tx_dma_channel_, tx_transfer);
  }
  if (result != ErrorCode::OK)
  {
    CancelDmaTransfer();
    return result;
  }
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

void SG200XSPI::DmaTxCallback(void* context, ErrorCode result, bool in_isr)
{
  static_cast<SG200XSPI*>(context)->OnDmaComplete(false, result, in_isr);
}

void SG200XSPI::DmaRxCallback(void* context, ErrorCode result, bool in_isr)
{
  static_cast<SG200XSPI*>(context)->OnDmaComplete(true, result, in_isr);
}

void SG200XSPI::OnDmaComplete(bool rx, ErrorCode result, bool in_isr)
{
  if (result != ErrorCode::OK)
  {
    FinishDma(result, in_isr);
    return;
  }
  if (circular_active_.load(std::memory_order_acquire))
  {
    if (!rx)
    {
      return;
    }
    result = CheckError();
    if (result != ErrorCode::OK)
    {
      FinishDma(result, in_isr);
      return;
    }
    // Use a snapshot because the user callback is allowed to stop the ring,
    // which clears active_op_ while this completion is still being dispatched.
    OperationRW operation = active_op_;
    operation.UpdateStatus(in_isr, ErrorCode::OK);
    return;
  }
  // In full-duplex master mode the RX DMA count is the terminal condition: a
  // received final frame proves the TX stream supplied every clock edge.  On
  // SG200x SSI the TX DMA completion status is not guaranteed to be raised
  // for every peripheral request, so waiting for it can leave an otherwise
  // completed transfer pending forever.  FinishDma still waits for SSI BUSY
  // to clear before exposing the result or releasing the DMA channels.
  if (rx)
  {
    FinishDma(ErrorCode::OK, in_isr);
  }
}

void SG200XSPI::FinishDma(ErrorCode result, bool in_isr)
{
  bool expected = false;
  if (!finishing_.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                          std::memory_order_acquire))
  {
    return;
  }
  if (!busy_.test(std::memory_order_acquire))
  {
    finishing_.store(false, std::memory_order_release);
    return;
  }
  if (result == ErrorCode::OK)
  {
    // RX DMA completing means the final frame reached memory, but it does not
    // by itself guarantee that the SSI has released BUSY. Preserve the final
    // CS hold time before disabling the controller, then inspect sticky SSI
    // errors that DMA descriptor completion alone cannot report.
    result = WaitForIdle();
    if (result == ErrorCode::OK)
    {
      result = CheckError();
    }
  }
  Register32(base_, REG_DMACR) = 0u;
  (void)Disable();
  if (rx_dma_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(rx_dma_channel_, in_isr);
  }
  if (tx_dma_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(tx_dma_channel_, in_isr);
  }
  rx_dma_channel_ = tx_dma_channel_ = 0xFFu;
  circular_active_.store(false, std::memory_order_release);
  if (result == ErrorCode::OK && active_read_needs_copy_ && active_read_.size_ != 0u)
  {
    const auto* source = static_cast<const uint8_t*>(GetRxBuffer().addr_);
    auto* destination = static_cast<uint8_t*>(active_read_.addr_);
    for (size_t index = 0u; index < active_read_.size_; ++index)
    {
      destination[index] = source[index + active_read_offset_];
    }
  }
  if (result == ErrorCode::OK)
  {
    SwitchBuffer();
  }
  if (active_op_.type == OperationRW::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(in_isr, result);
  }
  else
  {
    active_op_.UpdateStatus(in_isr, result);
  }
  active_op_ = {};
  active_read_ = {};
  active_read_offset_ = 0u;
  active_read_needs_copy_ = false;
  Unlock();
  finishing_.store(false, std::memory_order_release);
}

void SG200XSPI::CancelDmaTransfer()
{
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
  circular_active_.store(false, std::memory_order_release);
  if (active_op_.type == OperationRW::OperationType::BLOCK)
  {
    block_wait_.Cancel();
  }
  else if (active_op_.type == OperationRW::OperationType::POLLING)
  {
    *active_op_.data.status = OperationRW::OperationPollingStatus::ERROR;
  }
  active_op_ = {};
  active_read_ = {};
  active_read_offset_ = 0u;
  active_read_needs_copy_ = false;
  Unlock();
}

ErrorCode SG200XSPI::StartCircularTransfer(size_t size, OperationRW& op, bool in_isr)
{
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (op.type != OperationRW::OperationType::CALLBACK)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (size == 0u)
  {
    return ErrorCode::SIZE_ERR;
  }
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  if (size > rx.size_ || size > tx.size_)
  {
    return ErrorCode::SIZE_ERR;
  }
  return StartDmaTransfer({rx.addr_, size}, {tx.addr_, size}, op,
                          DmaBufferMode::INTERNAL, false, 0u,
                          SG200XDMAC::Mode::CIRCULAR);
}

ErrorCode SG200XSPI::StopCircularTransfer(bool in_isr)
{
  if (!IsValid())
  {
    return ErrorCode::ARG_ERR;
  }
  if (!circular_active_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }
  bool expected = false;
  if (!finishing_.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                          std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }

  // Keep SSI handshakes running while the channels consume their disable
  // requests. Stop TX first so RX can drain the final clocks without the TX
  // channel starting another self-linked block.
  ErrorCode result = ErrorCode::OK;
  if (tx_dma_channel_ != 0xFFu)
  {
    const ErrorCode release_result = SG200XDMAC::Release(tx_dma_channel_, in_isr);
    if (release_result == ErrorCode::OK)
    {
      tx_dma_channel_ = 0xFFu;
    }
    else
    {
      result = release_result;
    }
  }
  if (rx_dma_channel_ != 0xFFu)
  {
    const ErrorCode release_result = SG200XDMAC::Release(rx_dma_channel_, in_isr);
    if (release_result == ErrorCode::OK)
    {
      rx_dma_channel_ = 0xFFu;
    }
    else if (result == ErrorCode::OK)
    {
      result = release_result;
    }
  }
  Register32(base_, REG_DMACR) = 0u;
  (void)Disable();
  if (result != ErrorCode::OK)
  {
    finishing_.store(false, std::memory_order_release);
    return result;
  }
  circular_active_.store(false, std::memory_order_release);
  active_op_ = {};
  active_read_ = {};
  active_read_offset_ = 0u;
  active_read_needs_copy_ = false;
  Unlock();
  finishing_.store(false, std::memory_order_release);
  return ErrorCode::OK;
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

ErrorCode SG200XSPI::Read(RawData read_data, OperationRW& op, bool in_isr)
{
  return ReadAndWrite(read_data, {}, op, in_isr);
}

ErrorCode SG200XSPI::Write(ConstRawData write_data, OperationRW& op, bool in_isr)
{
  return ReadAndWrite({}, write_data, op, in_isr);
}

ErrorCode SG200XSPI::Transfer(size_t size, OperationRW& op, bool in_isr)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  if (size > rx.size_ || size > tx.size_)
  {
    return ErrorCode::SIZE_ERR;
  }
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return StartDmaTransfer({rx.addr_, size}, {tx.addr_, size}, op,
                          DmaBufferMode::INTERNAL);
}

ErrorCode SG200XSPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op,
                              bool in_isr)
{
  if (reg > 0xFFu)
  {
    return ErrorCode::SIZE_ERR;
  }
  if (write_data.size_ == 0u)
  {
    return ReadAndWrite({}, {}, op, in_isr);
  }
  if (write_data.addr_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return StartDmaTransfer({}, write_data, op, DmaBufferMode::STAGED, true,
                          static_cast<uint8_t>(reg & 0x7Fu));
}

ErrorCode SG200XSPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op,
                              bool in_isr)
{
  if (reg > 0xFFu)
  {
    return ErrorCode::SIZE_ERR;
  }
  if (read_data.size_ == 0u)
  {
    return ReadAndWrite({}, {}, op, in_isr);
  }
  if (read_data.addr_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  if (!IsValid() || in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  return StartDmaTransfer(read_data, {}, op, DmaBufferMode::STAGED, true,
                          static_cast<uint8_t>(reg | 0x80u));
}

}  // namespace LibXR

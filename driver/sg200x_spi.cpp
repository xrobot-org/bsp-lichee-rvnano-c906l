#include "sg200x_spi.hpp"

#include "sg200x_ll_csr.h"
#include "sg200x_ll_dmamux.h"
#include "sg200x_ll_gpio.h"

extern "C" int request_irq(unsigned int irqn, int (*handler)(int, void*),
                           unsigned long flags, const char* name, void* argument)
    __attribute__((weak));

namespace LibXR
{
namespace
{
#ifdef SPI2_HARDWARE_CS
constexpr bool SPI2_MANUAL_CS = false;
#else
constexpr bool SPI2_MANUAL_CS = true;
#endif

inline void ManualCsDelay() noexcept
{
  // SD1 pads live in the always-on power domain while SPI2 is clocked from
  // the main domain. Allow the synchronized CS level to settle before the
  // first clock and preserve a matching hold interval after the last clock.
  sgll_csr_delay_nops(1024u);
}

void ManualSpi2Cs(bool active) noexcept
{
  // The SD1 CS pad belongs to the always-on GPIO domain.
  sgll_gpio_output_enable(RTC_GPIO_REGS, PINMUX_SD1_D3_GPIO_MASK, true);
  sgll_gpio_output_write(RTC_GPIO_REGS, PINMUX_SD1_D3_GPIO_MASK, !active);
  sgll_csr_fence_io();
  ManualCsDelay();
}

bool IsC906LChannel(uint8_t channel) noexcept
{
  return channel < DMA_CHANNEL_COUNT &&
         (SG200XDMAC::OWNED_CHANNEL_MASK & SGLL_BIT(channel)) != 0u;
}
}  // namespace

SG200XSPI::SG200XSPI(Controller controller, uint8_t chip_select, RawData rx_buffer,
                     RawData tx_buffer, Configuration config, DmaChannels dma_channels)
    : SPI(rx_buffer, tx_buffer), dma_channels_(dma_channels)
{
  const uint8_t index = static_cast<uint8_t>(controller);
  if (index >= SPI_COUNT || chip_select >= SPI_CHIP_SELECT_COUNT ||
      rx_buffer.addr_ == nullptr || tx_buffer.addr_ == nullptr || rx_buffer.size_ == 0u ||
      tx_buffer.size_ == 0u ||
      (reinterpret_cast<uintptr_t>(rx_buffer.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(tx_buffer.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (rx_buffer.size_ % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (tx_buffer.size_ % SGLL_DCACHE_LINE_SIZE) != 0u)
  {
    return;
  }
  const auto peripheral = static_cast<SG200XRCC::PeripheralId>(
      static_cast<uint8_t>(SG200XRCC::PeripheralId::Spi0) + index);
  SG200XRCC& rcc = SG200XRCC::Instance();
  // The SSI can retain FIFO and serializer state across a C906 remoteproc
  // restart. Pulse its dedicated reset after enabling the clock path so a
  // previous Linux/U-Boot owner cannot leave the external SDO path latched in
  // an indeterminate state.
  if (rcc.ResetPeripheral(peripheral) != ErrorCode::OK)
  {
    return;
  }
  if (!IsC906LChannel(dma_channels.rx) || !IsC906LChannel(dma_channels.tx) ||
      dma_channels.rx == dma_channels.tx)
  {
    return;
  }
  input_clock_hz_ = rcc.ClockRate(SG200XRCC::ClockId::Spi);
  if (input_clock_hz_ == 0u)
  {
    return;
  }
  regs_ = sgll_spi_get(index);
  if (regs_ == nullptr)
  {
    return;
  }
  controller_index_ = index;
  chip_select_mask_ = SPI_SER_CS0_BIT;
  if (config.prescaler == Prescaler::UNKNOWN)
  {
    config.prescaler = Prescaler::DIV_2;
  }
  if (SetConfig(config) != ErrorCode::OK)
  {
    regs_ = nullptr;
    return;
  }
  // All DMA clients share one dispatcher: the SDK request_irq() replaces the
  // existing handler even with IRQF_SHARED. Registering a per-SPI handler
  // would lose I2C/UART completions on the same SDMA interrupt.
}

bool SG200XSPI::TryLock() noexcept
{
  return !busy_.test_and_set(std::memory_order_acquire);
}

void SG200XSPI::Unlock() noexcept { busy_.clear(std::memory_order_release); }

ErrorCode SG200XSPI::Disable() const
{
  sgll_spi_slave_select_disable(regs_);
  sgll_spi_disable(regs_);
  return ErrorCode::OK;
}

ErrorCode SG200XSPI::CheckError() const
{
  if ((sgll_spi_raw_interrupt_status_get(regs_) & SPI_RISR_ERROR_MASK) != 0u)
  {
    const uint32_t clear = sgll_spi_interrupt_clear(regs_);
    (void)clear;
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode SG200XSPI::WaitForIdle() const
{
  return sgll_spi_wait_idle(regs_, IDLE_WAIT_ATTEMPTS) ? ErrorCode::OK
                                                       : ErrorCode::TIMEOUT;
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
      (config.clock_phase != ClockPhase::EDGE_1 &&
       config.clock_phase != ClockPhase::EDGE_2))
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
  if (divider64 > SPI_BAUDR_MAX)
  {
    Unlock();
    return ErrorCode::NOT_SUPPORT;
  }
  const uint32_t divider = static_cast<uint32_t>(divider64);

  (void)Disable();
  const sgll_spi_mode_t spi_mode =
      config.clock_polarity == ClockPolarity::HIGH
          ? (config.clock_phase == ClockPhase::EDGE_2 ? SGLL_SPI_MODE_3 : SGLL_SPI_MODE_2)
          : (config.clock_phase == ClockPhase::EDGE_2 ? SGLL_SPI_MODE_1
                                                      : SGLL_SPI_MODE_0);
  uint32_t ctrlr0 = sgll_spi_ctrlr0_build_motorola_8bit(spi_mode, false);
#ifdef SPI_INTERNAL_LOOPBACK
  // SG2002 TRM CTRLR0[11]: route the TX shift-register output directly to RX.
  // This controller-level test mode intentionally bypasses the package pads.
  ctrlr0 = ctrlr0 | (SPI_CTRLR0_SRL_BIT);
#endif
  sgll_spi_ctrlr0_set(regs_, ctrlr0);
  if (!sgll_spi_baud_divider_set(regs_, divider) ||
      !sgll_spi_tx_fifo_threshold_set(regs_, 0u) ||
      !sgll_spi_rx_fifo_threshold_set(regs_, 0u))
  {
    Unlock();
    return ErrorCode::NOT_SUPPORT;
  }
  sgll_spi_interrupt_mask_set(regs_, 0u);
  sgll_spi_dma_disable(regs_);
  const uint32_t clear = sgll_spi_interrupt_clear(regs_);
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
                                      sgll_dma_mode_t dma_mode)
{
  if ((read_data.size_ != 0u && read_data.addr_ == nullptr) ||
      (write_data.size_ != 0u && write_data.addr_ == nullptr))
  {
    return ErrorCode::ARG_ERR;
  }
  if ((op.type == OperationRW::OperationType::CALLBACK && op.data.callback == nullptr) ||
      (op.type == OperationRW::OperationType::BLOCK && op.data.sem_info.sem == nullptr) ||
      (op.type == OperationRW::OperationType::POLLING && op.data.status == nullptr))
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

  const RawData internal_rx = GetRxBuffer();
  const RawData internal_tx = GetTxBuffer();
  RawData dma_rx = internal_rx;
  RawData dma_tx = internal_tx;
  const size_t dma_rx_capacity = internal_rx.size_;
  const size_t dma_tx_capacity = internal_tx.size_;
  if (buffer_mode == DmaBufferMode::INTERNAL)
  {
    dma_rx = read_data;
    dma_tx = {const_cast<void*>(write_data.addr_), write_data.size_};
    if (dma_rx.addr_ != internal_rx.addr_ || dma_tx.addr_ != internal_tx.addr_)
    {
      Unlock();
      return ErrorCode::ARG_ERR;
    }
  }
  if (dma_rx.addr_ == nullptr || dma_tx.addr_ == nullptr ||
      (reinterpret_cast<uintptr_t>(dma_rx.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(dma_tx.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
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
      destination[index + read_offset] = index < write_data.size_ ? source[index] : 0xFFu;
    }
  }

  const uintptr_t rx_start = reinterpret_cast<uintptr_t>(dma_rx.addr_);
  const uintptr_t tx_start = reinterpret_cast<uintptr_t>(dma_tx.addr_);

  ErrorCode result = SG200XDMAC::AcquireFixed(dma_channels_.rx);
  if (result == ErrorCode::OK)
  {
    rx_dma_channel_ = dma_channels_.rx;
    result = SG200XDMAC::AcquireFixed(dma_channels_.tx);
    if (result == ErrorCode::OK)
    {
      tx_dma_channel_ = dma_channels_.tx;
    }
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
  circular_active_.store(dma_mode == SGLL_DMA_MODE_CIRCULAR, std::memory_order_release);
  if (op.type == OperationRW::OperationType::BLOCK)
  {
    block_wait_.Start(*op.data.sem_info.sem);
  }
  // A DMA interrupt is allowed to preempt immediately after its channel is
  // armed. Publish RUNNING before that point so a fast transfer cannot leave
  // a POLLING operation in RUNNING after its completion has set it to DONE.
  op.MarkAsRunning();

  (void)Disable();
  // Request TX DMA before the SSI FIFO drains completely.  With a zero
  // watermark the controller can observe an empty FIFO between two DMA
  // handshakes and terminate a full-duplex transfer after the first 8-bit
  // frame (releasing CS while the descriptor still has data).  Keeping the
  // request threshold high leaves enough room for the next burst and holds
  // CS continuously across the complete fixed-size record.
  (void)sgll_spi_dma_tx_level_set(regs_, 4u);
  // RX requests remain per-frame; the RX FIFO is drained by the DMA channel.
  (void)sgll_spi_dma_rx_level_set(regs_, 0u);
  sgll_spi_dma_enable(regs_, true, true);
  sgll_spi_enable(regs_);
  sgll_spi_slave_select_set(regs_, chip_select_mask_);
  const SG200XDMAC::Transfer rx_transfer{
      .memory = rx_start,
      .memory_capacity = dma_rx_capacity,
      .peripheral = sgll_spi_data_address(regs_),
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          sgll_dmamux_spi_request_get(controller_index_, false)),
      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
      .width = SG200XDMAC::Width::BYTE,
      .mode = static_cast<SG200XDMAC::Mode>(dma_mode),
      .callback = &DmaRxCallback,
      .context = this};
  const SG200XDMAC::Transfer tx_transfer{
      .memory = tx_start,
      .memory_capacity = dma_tx_capacity,
      .peripheral = sgll_spi_data_address(regs_),
      .count = frames,
      .request = static_cast<SG200XDMAC::Request>(
          sgll_dmamux_spi_request_get(controller_index_, true)),
      .direction = SG200XDMAC::Direction::MEMORY_TO_PERIPHERAL,
      .width = SG200XDMAC::Width::BYTE,
      .mode = static_cast<SG200XDMAC::Mode>(dma_mode),
      .callback = &DmaTxCallback,
      .context = this};
  result = SG200XDMAC::Start(rx_dma_channel_, rx_transfer);
  if (result == ErrorCode::OK)
  {
    if (SPI2_MANUAL_CS && regs_ == SPI2_REGS)
    {
      ManualSpi2Cs(true);
    }
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
  if (rx && in_isr && result == ErrorCode::OK)
  {
    dma_receive_count_.fetch_add(1u, std::memory_order_relaxed);
  }
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
  sgll_spi_dma_disable(regs_);
  (void)Disable();
  if (SPI2_MANUAL_CS && regs_ == SPI2_REGS)
  {
    ManualSpi2Cs(false);
  }
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
  OperationRW operation = active_op_;
  active_op_ = {};
  active_read_ = {};
  active_read_offset_ = 0u;
  active_read_needs_copy_ = false;
  Unlock();
  finishing_.store(false, std::memory_order_release);
  if (operation.type == OperationRW::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(in_isr, result);
  }
  else
  {
    operation.UpdateStatus(in_isr, result);
  }
}

void SG200XSPI::CancelDmaTransfer()
{
  sgll_spi_dma_disable(regs_);
  (void)Disable();
  if (SPI2_MANUAL_CS && regs_ == SPI2_REGS)
  {
    ManualSpi2Cs(false);
  }
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
  return StartDmaTransfer({rx.addr_, size}, {tx.addr_, size}, op, DmaBufferMode::INTERNAL,
                          false, 0u, SGLL_DMA_MODE_CIRCULAR);
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
  sgll_spi_dma_disable(regs_);
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
  // LibXR SPI is DMA-only on SG2002. Keep ordinary API calls on the same
  // path as Transfer() so buffer/cache/CS behavior cannot diverge.
  return StartDmaTransfer(read_data, write_data, op, DmaBufferMode::STAGED);
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
  if (write_data.size_ != 0u && write_data.addr_ == nullptr)
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
  if (read_data.size_ != 0u && read_data.addr_ == nullptr)
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

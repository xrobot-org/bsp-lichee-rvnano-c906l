#include "sg200x_i2c.hpp"

#include "sg200x_ll_csr.h"
#include "sg200x_ll_dmamux.h"
#include "sg200x_ll_plic.h"

extern "C" int request_irq(unsigned int, int (*)(int, void*), unsigned long, const char*,
                           void*) __attribute__((weak));

namespace LibXR
{

std::atomic<bool> SG200XI2C::controller_claimed_[CONTROLLER_COUNT]{};
std::atomic<SG200XI2C*> SG200XI2C::instances_[CONTROLLER_COUNT]{};
bool SG200XI2C::irq_registered_[CONTROLLER_COUNT]{};

SG200XI2C::SG200XI2C(Controller controller, RawData tx_command_buffer, RawData rx_buffer,
                     Configuration config, DmaChannels dma_channels)
    : tx_stage_(tx_command_buffer), rx_stage_(rx_buffer), dma_channels_(dma_channels)
{
  const auto index = static_cast<uint8_t>(controller);
  if (index >= CONTROLLER_COUNT || tx_stage_.addr_ == nullptr ||
      rx_stage_.addr_ == nullptr || tx_stage_.size_ < sizeof(uint16_t) ||
      rx_stage_.size_ < sizeof(uint16_t) ||
      (reinterpret_cast<uintptr_t>(tx_stage_.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(rx_stage_.addr_) % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (tx_stage_.size_ % SGLL_DCACHE_LINE_SIZE) != 0u ||
      (rx_stage_.size_ % SGLL_DCACHE_LINE_SIZE) != 0u || request_irq == nullptr ||
      !detail::IsOwnedChannel(dma_channels.tx) ||
      !detail::IsOwnedChannel(dma_channels.rx) || dma_channels.tx == dma_channels.rx)
  {
    return;
  }
  bool unclaimed = false;
  if (!controller_claimed_[index].compare_exchange_strong(
          unclaimed, true, std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return;
  }
  controller_index_ = index;
  const auto peripheral = static_cast<SG200XRCC::PeripheralId>(
      static_cast<uint8_t>(SG200XRCC::PeripheralId::I2c0) + index);
  SG200XRCC& rcc = SG200XRCC::Instance();
  if (rcc.PreparePeripheral(peripheral) != ErrorCode::OK)
  {
    controller_index_ = 0xFFu;
    controller_claimed_[index].store(false, std::memory_order_release);
    return;
  }
  input_clock_hz_ = rcc.ClockRate(SG200XRCC::ClockId::I2c);
  if (input_clock_hz_ == 0u)
  {
    controller_index_ = 0xFFu;
    controller_claimed_[index].store(false, std::memory_order_release);
    return;
  }
  regs_ = sgll_i2c_get(index);
  if (SetConfig(config) != ErrorCode::OK)
  {
    regs_ = nullptr;
    controller_index_ = 0xFFu;
    controller_claimed_[index].store(false, std::memory_order_release);
    return;
  }

  bool registration_ok = true;
  instances_[index].store(this, std::memory_order_release);
  if (!irq_registered_[index])
  {
    if (request_irq(IRQ0 + index, &Interrupt, 0u, "sg200x-i2c", nullptr) != 0)
    {
      registration_ok = false;
    }
    else
    {
      // C906L reset does not reset the external PLIC context. Retire a claim
      // left in service if remoteproc stopped the previous image in this ISR.
      // The initialized instance was published before request_irq(), so an
      // interrupt arriving in this window can be dispatched without masking
      // machine interrupts.
      sgll_plic_irq_complete(IRQ0 + index);
      sgll_csr_fence_io();
      irq_registered_[index] = true;
    }
  }
  if (!registration_ok)
  {
    SG200XI2C* expected = this;
    (void)instances_[index].compare_exchange_strong(
        expected, nullptr, std::memory_order_acq_rel, std::memory_order_acquire);
    regs_ = nullptr;
    controller_index_ = 0xFFu;
    controller_claimed_[index].store(false, std::memory_order_release);
  }
}

ErrorCode SG200XI2C::Enable(bool enabled) const
{
  return sgll_i2c_enable_wait(regs_, enabled, ENABLE_WAIT_ATTEMPTS) ? ErrorCode::OK
                                                                    : ErrorCode::TIMEOUT;
}

ErrorCode SG200XI2C::SetConfig(Configuration config)
{
  if (regs_ == nullptr)
  {
    return ErrorCode::ARG_ERR;
  }
  if (faulted_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }
  bool inactive = false;
  if (!active_.compare_exchange_strong(inactive, true, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }
  if (config.clock_speed != 100000u && config.clock_speed != 400000u)
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }
  sgll_i2c_timing_t timing;
  if (!sgll_i2c_timing_calculate(input_clock_hz_, &timing))
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }
  sgll_i2c_init_t init;
  sgll_i2c_struct_init(&init);
  init.speed = config.clock_speed == 400000u ? I2C_SPEED_FAST : I2C_SPEED_STANDARD;
  ErrorCode result = Enable(false);
  if (result == ErrorCode::OK)
  {
    result = sgll_i2c_init_with_timing(regs_, &init, &timing) ? Enable(true)
                                                              : ErrorCode::STATE_ERR;
  }
  active_.store(false, std::memory_order_release);
  return result;
}

ErrorCode SG200XI2C::Start(uint16_t slave, const uint8_t* prefix, size_t prefix_size,
                           const uint8_t* write, size_t write_size, RawData read,
                           ReadOperation* read_op, WriteOperation* write_op, bool in_isr)
{
  if (in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (regs_ == nullptr || slave > I2C_TAR_ADDRESS_MASK ||
      (read.size_ != 0u && read.addr_ == nullptr) ||
      (prefix_size != 0u && prefix == nullptr) ||
      (write_size != 0u && write == nullptr) ||
      (read_op == nullptr) == (write_op == nullptr))
  {
    return ErrorCode::ARG_ERR;
  }
  if (faulted_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }
  Operation<ErrorCode>& requested_operation = read_op != nullptr ? *read_op : *write_op;
  if ((requested_operation.type == Operation<ErrorCode>::OperationType::CALLBACK &&
       requested_operation.data.callback == nullptr) ||
      (requested_operation.type == Operation<ErrorCode>::OperationType::BLOCK &&
       requested_operation.data.sem_info.sem == nullptr) ||
      (requested_operation.type == Operation<ErrorCode>::OperationType::POLLING &&
       requested_operation.data.status == nullptr))
  {
    return ErrorCode::ARG_ERR;
  }
  bool inactive = false;
  if (!active_.compare_exchange_strong(inactive, true, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }
  if (finishing_.load(std::memory_order_acquire))
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::BUSY;
  }
  if (prefix_size == 0u && write_size == 0u && read.size_ == 0u)
  {
    active_.store(false, std::memory_order_release);
    if (requested_operation.type != Operation<ErrorCode>::OperationType::BLOCK)
    {
      requested_operation.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }
  const size_t tx_capacity = tx_stage_.size_ / sizeof(uint16_t);
  const size_t rx_capacity = rx_stage_.size_ / sizeof(uint16_t);
  if (prefix_size > tx_capacity || write_size > tx_capacity - prefix_size ||
      read.size_ > tx_capacity - prefix_size - write_size || read.size_ > rx_capacity)
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::SIZE_ERR;
  }
  const size_t commands = prefix_size + write_size + read.size_;
  auto* cmd = static_cast<uint16_t*>(tx_stage_.addr_);
  for (size_t i = 0; i < prefix_size; ++i)
  {
    cmd[i] = static_cast<uint16_t>(
        sgll_i2c_data_command_build(prefix[i], false, i + 1u == commands, false));
  }
  for (size_t i = 0; i < write_size; ++i)
  {
    cmd[prefix_size + i] = static_cast<uint16_t>(sgll_i2c_data_command_build(
        write[i], false, prefix_size + i + 1u == commands, false));
  }
  for (size_t i = 0; i < read.size_; ++i)
  {
    cmd[prefix_size + write_size + i] = static_cast<uint16_t>(sgll_i2c_data_command_build(
        0u, true, i + 1u == read.size_, i == 0u && prefix_size + write_size != 0u));
  }
  ErrorCode result = SG200XDMAC::AcquireFixed(dma_channels_.tx);
  if (result == ErrorCode::OK)
  {
    tx_channel_ = dma_channels_.tx;
  }
  if (result == ErrorCode::OK && read.size_)
  {
    result = SG200XDMAC::AcquireFixed(dma_channels_.rx);
    if (result == ErrorCode::OK)
    {
      rx_channel_ = dma_channels_.rx;
    }
  }
  if (result != ErrorCode::OK)
  {
    if (tx_channel_ != 0xFFu)
    {
      const ErrorCode release_result = SG200XDMAC::Release(tx_channel_);
      if (release_result == ErrorCode::OK)
      {
        tx_channel_ = 0xFFu;
      }
      else
      {
        result = release_result;
        faulted_.store(true, std::memory_order_release);
      }
    }
    active_.store(false, std::memory_order_release);
    return result;
  }
  read_target_ = read;
  operation_ = requested_operation;
  tx_done_ = false;
  rx_done_ = read.size_ == 0u;
  stop_done_ = false;
  const bool block = operation_.type == Operation<ErrorCode>::OperationType::BLOCK;
  const uint32_t block_timeout = block ? operation_.data.sem_info.timeout : 0u;
  if (block)
  {
    block_wait_.Start(*operation_.data.sem_info.sem);
  }
  result = Enable(false);
  if (result != ErrorCode::OK)
  {
    Complete(result, false, false);
    return result;
  }
  sgll_i2c_master_address_set(regs_, slave, slave > I2C_TAR_7BIT_ADDRESS_MASK);
  sgll_i2c_dma_threshold_set(regs_, 0u, 0u);
  sgll_i2c_interrupt_mask_set(regs_, I2C_INTR_TX_ABRT_BIT | I2C_INTR_STOP_DET_BIT);
  (void)sgll_i2c_interrupt_clear(regs_);
  sgll_i2c_dma_enable(regs_, read.size_ != 0u, true);
  result = Enable(true);
  if (result != ErrorCode::OK)
  {
    Complete(result, false, false);
    return result;
  }
  operation_.MarkAsRunning();
  if (read.size_)
  {
    result = SG200XDMAC::Start(
        rx_channel_, {.memory = reinterpret_cast<uintptr_t>(rx_stage_.addr_),
                      .memory_capacity = rx_stage_.size_,
                      .peripheral = sgll_i2c_data_address(regs_),
                      .count = read.size_,
                      .request = static_cast<SG200XDMAC::Request>(
                          sgll_dmamux_i2c_request_get(controller_index_, false)),
                      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
                      .width = SG200XDMAC::Width::HALF_WORD,
                      .mode = SG200XDMAC::Mode::NORMAL,
                      .callback = &DmaRx,
                      .context = this});
  }
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Start(tx_channel_,
                               {.memory = reinterpret_cast<uintptr_t>(tx_stage_.addr_),
                                .memory_capacity = tx_stage_.size_,
                                .peripheral = sgll_i2c_data_address(regs_),
                                .count = commands,
                                .request = static_cast<SG200XDMAC::Request>(
                                    sgll_dmamux_i2c_request_get(controller_index_, true)),
                                .direction = SG200XDMAC::Direction::MEMORY_TO_PERIPHERAL,
                                .width = SG200XDMAC::Width::HALF_WORD,
                                .mode = SG200XDMAC::Mode::NORMAL,
                                .callback = &DmaTx,
                                .context = this});
  }
  if (result != ErrorCode::OK)
  {
    Complete(result, false, false);
    return result;
  }
  if (!block)
  {
    return ErrorCode::OK;
  }
  const ErrorCode wait_result = block_wait_.Wait(block_timeout);
  if (wait_result == ErrorCode::TIMEOUT)
  {
    Complete(ErrorCode::TIMEOUT);
  }
  return wait_result;
}

void SG200XI2C::DmaTx(void* c, ErrorCode r, bool in_isr)
{
  static_cast<SG200XI2C*>(c)->OnDma(false, r, in_isr);
}

void SG200XI2C::DmaRx(void* c, ErrorCode r, bool in_isr)
{
  static_cast<SG200XI2C*>(c)->OnDma(true, r, in_isr);
}

void SG200XI2C::OnDma(bool rx, ErrorCode result, bool in_isr)
{
  if (result != ErrorCode::OK)
  {
    Complete(result, in_isr);
  }
  else
  {
    if (rx)
    {
      rx_done_ = true;
    }
    else
    {
      tx_done_ = true;
    }
    if (tx_done_ && rx_done_ && stop_done_)
    {
      Complete(ErrorCode::OK, in_isr);
    }
  }
}

int SG200XI2C::Interrupt(int irq, void*)
{
  if (irq < static_cast<int>(IRQ0) || irq >= static_cast<int>(IRQ0 + CONTROLLER_COUNT))
  {
    return 0;
  }
  SG200XI2C* self =
      instances_[static_cast<uint8_t>(irq - IRQ0)].load(std::memory_order_acquire);
  if (self == nullptr || self->regs_ == nullptr)
  {
    return 0;
  }
  const uint32_t raw = sgll_i2c_raw_interrupt_status_get(self->regs_);
  if (raw & I2C_INTR_TX_ABRT_BIT)
  {
    const uint32_t x = sgll_i2c_abort_clear(self->regs_);
    (void)x;
    self->Complete(ErrorCode::NO_RESPONSE, true);
  }
  if (raw & I2C_INTR_STOP_DET_BIT)
  {
    const uint32_t x = sgll_i2c_stop_clear(self->regs_);
    (void)x;
    self->stop_done_ = true;
    if (self->tx_done_ && self->rx_done_)
    {
      self->Complete(ErrorCode::OK, true);
    }
  }
  return 0;
}

void SG200XI2C::Complete(ErrorCode result, bool in_isr, bool notify_operation)
{
  bool expected = false;
  if (!finishing_.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                                          std::memory_order_acquire))
  {
    return;
  }
  if (!active_.load(std::memory_order_acquire))
  {
    finishing_.store(false, std::memory_order_release);
    return;
  }
  sgll_i2c_dma_disable(regs_);
  sgll_i2c_interrupt_mask_set(regs_, 0u);
  if (tx_channel_ != 0xFFu)
  {
    const ErrorCode release_result = SG200XDMAC::Release(tx_channel_, in_isr);
    if (release_result == ErrorCode::OK)
    {
      tx_channel_ = 0xFFu;
    }
    else
    {
      result = release_result;
      faulted_.store(true, std::memory_order_release);
    }
  }
  if (rx_channel_ != 0xFFu)
  {
    const ErrorCode release_result = SG200XDMAC::Release(rx_channel_, in_isr);
    if (release_result == ErrorCode::OK)
    {
      rx_channel_ = 0xFFu;
    }
    else
    {
      result = release_result;
      faulted_.store(true, std::memory_order_release);
    }
  }
  if (result != ErrorCode::OK)
  {
    (void)Enable(false);
    const uint32_t clear = sgll_i2c_interrupt_clear(regs_);
    (void)clear;
    (void)Enable(true);
  }
  if (result == ErrorCode::OK && read_target_.size_)
  {
    auto* d = static_cast<uint8_t*>(read_target_.addr_);
    auto* s = static_cast<uint16_t*>(rx_stage_.addr_);
    for (size_t i = 0; i < read_target_.size_; ++i)
    {
      d[i] = static_cast<uint8_t>(s[i]);
    }
  }
  Operation<ErrorCode> operation = operation_;
  operation_ = {};
  read_target_ = {};
  if (!notify_operation)
  {
    if (operation.type == Operation<ErrorCode>::OperationType::BLOCK)
    {
      block_wait_.Cancel();
    }
    else if (operation.type == Operation<ErrorCode>::OperationType::POLLING)
    {
      *operation.data.status = Operation<ErrorCode>::OperationPollingStatus::READY;
    }
    active_.store(false, std::memory_order_release);
    finishing_.store(false, std::memory_order_release);
    return;
  }
  if (operation.type == Operation<ErrorCode>::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(in_isr, result);
    active_.store(false, std::memory_order_release);
    finishing_.store(false, std::memory_order_release);
  }
  else
  {
    // Publish the idle state before external callback/status observers run.
    active_.store(false, std::memory_order_release);
    finishing_.store(false, std::memory_order_release);
    operation.UpdateStatus(in_isr, result);
  }
}

ErrorCode SG200XI2C::Read(uint16_t a, RawData d, ReadOperation& o, bool isr)
{
  return Start(a, nullptr, 0, nullptr, 0, d, &o, nullptr, isr);
}

ErrorCode SG200XI2C::Write(uint16_t a, ConstRawData d, WriteOperation& o, bool isr)
{
  return Start(a, nullptr, 0, static_cast<const uint8_t*>(d.addr_), d.size_, {}, nullptr,
               &o, isr);
}

ErrorCode SG200XI2C::MemRead(uint16_t a, uint16_t m, RawData d, ReadOperation& o,
                             MemAddrLength l, bool isr)
{
  if (l != MemAddrLength::BYTE_8 && l != MemAddrLength::BYTE_16)
  {
    return ErrorCode::ARG_ERR;
  }
  uint8_t p[2] = {static_cast<uint8_t>(m >> 8), static_cast<uint8_t>(m)};
  const size_t n = l == MemAddrLength::BYTE_16 ? 2 : 1;
  return Start(a, p + 2 - n, n, nullptr, 0, d, &o, nullptr, isr);
}

ErrorCode SG200XI2C::MemWrite(uint16_t a, uint16_t m, ConstRawData d, WriteOperation& o,
                              MemAddrLength l, bool isr)
{
  if (l != MemAddrLength::BYTE_8 && l != MemAddrLength::BYTE_16)
  {
    return ErrorCode::ARG_ERR;
  }
  uint8_t p[2] = {static_cast<uint8_t>(m >> 8), static_cast<uint8_t>(m)};
  const size_t n = l == MemAddrLength::BYTE_16 ? 2 : 1;
  return Start(a, p + 2 - n, n, static_cast<const uint8_t*>(d.addr_), d.size_, {},
               nullptr, &o, isr);
}
}  // namespace LibXR

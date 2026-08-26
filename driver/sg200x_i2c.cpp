#include "sg200x_i2c.hpp"

#include "sg200x_mmio.hpp"

extern "C" int request_irq(unsigned int, int (*)(int, void*), unsigned long, const char*,
                           void*) __attribute__((weak));

namespace LibXR
{
SG200XI2C::SG200XI2C(Controller controller, RawData tx_command_buffer, RawData rx_buffer,
                     uint32_t input_clock_hz, Configuration config)
    : input_clock_hz_(input_clock_hz), tx_stage_(tx_command_buffer), rx_stage_(rx_buffer)
{
  const auto index = static_cast<uint8_t>(controller);
  if (index >= 5u || tx_stage_.addr_ == nullptr || rx_stage_.addr_ == nullptr ||
      (reinterpret_cast<uintptr_t>(tx_stage_.addr_) & 1u) != 0u ||
      (reinterpret_cast<uintptr_t>(rx_stage_.addr_) & 1u) != 0u || request_irq == nullptr)
    return;
  base_ = I2C0_BASE + static_cast<uintptr_t>(index) * STRIDE;
  if (request_irq(IRQ0 + index, &Interrupt, 0u, "sg200x-i2c", this) != 0 ||
      SetConfig(config) != ErrorCode::OK)
  {
    base_ = 0u;
  }
}

ErrorCode SG200XI2C::Enable(bool enabled) const
{
  Register32(base_, REG_ENABLE) = enabled ? 1u : 0u;
  for (uint32_t i = 0u; i < 100000u; ++i)
  {
    if ((Register32(base_, REG_ENABLE_STATUS) & 1u) == (enabled ? 1u : 0u))
    {
      return ErrorCode::OK;
    }
  }
  return ErrorCode::TIMEOUT;
}

ErrorCode SG200XI2C::SetConfig(Configuration config)
{
  bool inactive = false;
  if (base_ == 0u ||
      !active_.compare_exchange_strong(inactive, true, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
  {
    return ErrorCode::BUSY;
  }
  if ((config.clock_speed != 100000u && config.clock_speed != 400000u) ||
      (input_clock_hz_ != 25000000u && input_clock_hz_ != 100000000u))
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }
  ErrorCode result = Enable(false);
  if (result == ErrorCode::OK)
  {
    Register32(base_, REG_CON) = CON_MASTER | CON_RESTART | CON_SLAVE_DISABLE |
                                 (config.clock_speed == 400000u ? CON_FS : CON_SS);
    const bool c25 = input_clock_hz_ == 25000000u;
    Register32(base_, REG_SS_H) = c25 ? 115u : 460u;
    Register32(base_, REG_SS_L) = c25 ? 135u : 540u;
    Register32(base_, REG_FS_H) = c25 ? 21u : 90u;
    Register32(base_, REG_FS_L) = c25 ? 42u : 160u;
    Register32(base_, REG_SDA_HOLD) = 1u;
    Register32(base_, REG_SDA_SETUP) = c25 ? 6u : 25u;
    Register32(base_, REG_SPKLEN) = c25 ? 2u : 5u;
    Register32(base_, REG_INTR_MASK) = 0u;
    Register32(base_, REG_DMA_CR) = 0u;
    const uint32_t clear = Register32(base_, REG_CLR_INTR);
    (void)clear;
    result = Enable(true);
  }
  active_.store(false, std::memory_order_release);
  return result;
}

ErrorCode SG200XI2C::Start(uint16_t slave, const uint8_t* prefix, size_t prefix_size,
                           const uint8_t* write, size_t write_size, RawData read,
                           ReadOperation* read_op, WriteOperation* write_op, bool in_isr)
{
  bool inactive = false;
  if (in_isr || base_ == 0u || slave > 0x3FFu || (read.size_ && !read.addr_) ||
      (prefix_size && !prefix) || (write_size && !write) ||
      !active_.compare_exchange_strong(inactive, true, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
  {
    return in_isr ? ErrorCode::NOT_SUPPORT : ErrorCode::BUSY;
  }
  const size_t commands = prefix_size + write_size + read.size_;
  if (commands == 0u)
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::OK;
  }
  if (commands * sizeof(uint16_t) > tx_stage_.size_ ||
      read.size_ * sizeof(uint16_t) > rx_stage_.size_)
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::SIZE_ERR;
  }
  auto* cmd = static_cast<uint16_t*>(tx_stage_.addr_);
  for (size_t i = 0; i < prefix_size; ++i)
  {
    cmd[i] = prefix[i];
  }
  for (size_t i = 0; i < write_size; ++i)
  {
    cmd[prefix_size + i] = write[i];
  }
  for (size_t i = 0; i < read.size_; ++i)
    cmd[prefix_size + write_size + i] =
        CMD_READ | ((i == 0 && prefix_size + write_size) ? CMD_RESTART : 0u);
  cmd[commands - 1u] |= CMD_STOP;
  ErrorCode result = SG200XDMAC::Acquire(tx_channel_);
  if (result == ErrorCode::OK && read.size_)
  {
    result = SG200XDMAC::Acquire(rx_channel_);
  }
  if (result != ErrorCode::OK)
  {
    if (tx_channel_ != 0xFFu)
    {
      SG200XDMAC::Release(tx_channel_);
    }
    active_.store(false, std::memory_order_release);
    return result;
  }
  read_target_ = read;
  read_op_ = read_op;
  write_op_ = write_op;
  tx_done_ = false;
  rx_done_ = read.size_ == 0u;
  stop_done_ = false;
  if ((read_op && read_op->type == ReadOperation::OperationType::BLOCK) ||
      (write_op && write_op->type == WriteOperation::OperationType::BLOCK))
  {
    auto* op = read_op ? static_cast<Operation<ErrorCode>*>(read_op)
                       : static_cast<Operation<ErrorCode>*>(write_op);
    block_wait_.Start(*op->data.sem_info.sem);
  }
  result = Enable(false);
  if (result != ErrorCode::OK)
  {
    Complete(result);
    return result;
  }
  uint32_t con = Register32(base_, REG_CON);
  Register32(base_, REG_CON) = slave > 0x7Fu ? con | CON_10B : con & ~CON_10B;
  Register32(base_, REG_TAR) = slave;
  Register32(base_, REG_DMA_TDLR) = 0u;
  Register32(base_, REG_DMA_RDLR) = 0u;
  Register32(base_, REG_INTR_MASK) = INTR_ABRT | INTR_STOP;
  const uint32_t clear = Register32(base_, REG_CLR_INTR);
  (void)clear;
  Register32(base_, REG_DMA_CR) = read.size_ ? 3u : 2u;
  result = Enable(true);
  if (result != ErrorCode::OK)
  {
    Complete(result);
    return result;
  }
  const auto idx = static_cast<uint8_t>((base_ - I2C0_BASE) / STRIDE);
  if (read.size_)
  {
    result = SG200XDMAC::Start(
        rx_channel_, {.memory = reinterpret_cast<uintptr_t>(rx_stage_.addr_),
                      .peripheral = base_ + REG_DATA_CMD,
                      .count = read.size_,
                      .request = static_cast<SG200XDMAC::Request>(
                          static_cast<uint8_t>(SG200XDMAC::Request::I2C0_RX) + idx * 2u),
                      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
                      .width = SG200XDMAC::Width::HALF_WORD,
                      .callback = &DmaRx,
                      .context = this});
  }
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Start(
        tx_channel_, {.memory = reinterpret_cast<uintptr_t>(tx_stage_.addr_),
                      .peripheral = base_ + REG_DATA_CMD,
                      .count = commands,
                      .request = static_cast<SG200XDMAC::Request>(
                          static_cast<uint8_t>(SG200XDMAC::Request::I2C0_TX) + idx * 2u),
                      .direction = SG200XDMAC::Direction::MEMORY_TO_PERIPHERAL,
                      .width = SG200XDMAC::Width::HALF_WORD,
                      .callback = &DmaTx,
                      .context = this});
  }
  if (result != ErrorCode::OK)
  {
    Complete(result);
    return result;
  }
  if (read_op)
  {
    read_op->MarkAsRunning();
  }
  if (write_op)
  {
    write_op->MarkAsRunning();
  }
  auto* op = read_op ? static_cast<Operation<ErrorCode>*>(read_op)
                     : static_cast<Operation<ErrorCode>*>(write_op);
  if (op->type != Operation<ErrorCode>::OperationType::BLOCK)
  {
    return ErrorCode::OK;
  }
  const ErrorCode wait_result = block_wait_.Wait(op->data.sem_info.timeout);
  if (wait_result == ErrorCode::TIMEOUT)
  {
    Complete(ErrorCode::TIMEOUT);
  }
  return wait_result;
}

void SG200XI2C::DmaTx(void* c, ErrorCode r)
{
  static_cast<SG200XI2C*>(c)->OnDma(false, r);
}

void SG200XI2C::DmaRx(void* c, ErrorCode r)
{
  static_cast<SG200XI2C*>(c)->OnDma(true, r);
}

void SG200XI2C::OnDma(bool rx, ErrorCode result)
{
  if (result != ErrorCode::OK)
  {
    Complete(result);
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
      Complete(ErrorCode::OK);
    }
  }
}

int SG200XI2C::Interrupt(int, void* c)
{
  auto* self = static_cast<SG200XI2C*>(c);
  const uint32_t raw = Register32(self->base_, REG_RAW);
  if (raw & INTR_ABRT)
  {
    const uint32_t x = Register32(self->base_, REG_CLR_ABRT);
    (void)x;
    self->Complete(ErrorCode::NO_RESPONSE);
  }
  if (raw & INTR_STOP)
  {
    const uint32_t x = Register32(self->base_, REG_CLR_STOP);
    (void)x;
    self->stop_done_ = true;
    if (self->tx_done_ && self->rx_done_)
    {
      self->Complete(ErrorCode::OK);
    }
  }
  return 0;
}

void SG200XI2C::Complete(ErrorCode result)
{
  if (!active_.exchange(false, std::memory_order_acq_rel))
  {
    return;
  }
  Register32(base_, REG_DMA_CR) = 0u;
  Register32(base_, REG_INTR_MASK) = 0u;
  if (tx_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(tx_channel_);
  }
  if (rx_channel_ != 0xFFu)
  {
    SG200XDMAC::Release(rx_channel_);
  }
  tx_channel_ = rx_channel_ = 0xFFu;
  if (result != ErrorCode::OK)
  {
    (void)Enable(false);
    const uint32_t clear = Register32(base_, REG_CLR_INTR);
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
  auto* op = read_op_ ? static_cast<Operation<ErrorCode>*>(read_op_)
                      : static_cast<Operation<ErrorCode>*>(write_op_);
  read_op_ = nullptr;
  write_op_ = nullptr;
  read_target_ = {};
  if (op->type == Operation<ErrorCode>::OperationType::BLOCK)
  {
    (void)block_wait_.TryPost(true, result);
  }
  else
  {
    op->UpdateStatus(true, result);
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
  uint8_t p[2] = {static_cast<uint8_t>(m >> 8), static_cast<uint8_t>(m)};
  const size_t n = l == MemAddrLength::BYTE_16 ? 2 : 1;
  return Start(a, p + 2 - n, n, nullptr, 0, d, &o, nullptr, isr);
}

ErrorCode SG200XI2C::MemWrite(uint16_t a, uint16_t m, ConstRawData d, WriteOperation& o,
                              MemAddrLength l, bool isr)
{
  uint8_t p[2] = {static_cast<uint8_t>(m >> 8), static_cast<uint8_t>(m)};
  const size_t n = l == MemAddrLength::BYTE_16 ? 2 : 1;
  return Start(a, p + 2 - n, n, static_cast<const uint8_t*>(d.addr_), d.size_, {},
               nullptr, &o, isr);
}
}  // namespace LibXR

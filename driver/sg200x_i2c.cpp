#include "sg200x_i2c.hpp"

#include "sg200x_mmio.hpp"

extern "C" int request_irq(unsigned int, int (*)(int, void*), unsigned long, const char*,
                           void*) __attribute__((weak));

namespace LibXR
{
namespace
{
constexpr uint32_t NANOSECONDS_PER_SECOND = 1000000000u;
constexpr uint32_t STANDARD_HIGH_NS = 4000u;
constexpr uint32_t STANDARD_LOW_NS = 4700u;
constexpr uint32_t FAST_HIGH_NS = 600u;
constexpr uint32_t FAST_LOW_NS = 1300u;
constexpr uint32_t SCL_FALL_NS = 300u;
constexpr uint32_t SDA_HOLD_NS = 300u;
constexpr uint32_t SDA_SETUP_NS = 1000u;
constexpr uint32_t SPIKE_SUPPRESSION_NS = 50u;
constexpr uintptr_t PLIC_CLAIM_COMPLETE = 0x70200004u;

[[nodiscard]] constexpr uint32_t ClockCyclesForNanoseconds(uint32_t clock_hz,
                                                           uint32_t nanoseconds)
{
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(clock_hz) * nanoseconds + NANOSECONDS_PER_SECOND - 1u) /
      NANOSECONDS_PER_SECOND);
}

[[nodiscard]] constexpr uint32_t SclHighCount(uint32_t clock_hz,
                                              uint32_t high_ns)
{
  const uint32_t cycles =
      ClockCyclesForNanoseconds(clock_hz, high_ns + SCL_FALL_NS);
  return cycles > 3u ? cycles - 3u : 0u;
}

[[nodiscard]] constexpr uint32_t SclLowCount(uint32_t clock_hz, uint32_t low_ns)
{
  const uint32_t cycles = ClockCyclesForNanoseconds(clock_hz, low_ns + SCL_FALL_NS);
  return cycles > 1u ? cycles - 1u : 0u;
}
}  // namespace

std::atomic<bool> SG200XI2C::controller_claimed_[CONTROLLER_COUNT]{};
std::atomic<SG200XI2C*> SG200XI2C::instances_[CONTROLLER_COUNT]{};
bool SG200XI2C::irq_registered_[CONTROLLER_COUNT]{};

SG200XI2C::SG200XI2C(Controller controller, RawData tx_command_buffer, RawData rx_buffer,
                     Configuration config)
    : tx_stage_(tx_command_buffer), rx_stage_(rx_buffer)
{
  const auto index = static_cast<uint8_t>(controller);
  if (index >= CONTROLLER_COUNT || tx_stage_.addr_ == nullptr ||
      rx_stage_.addr_ == nullptr ||
      tx_stage_.size_ < sizeof(uint16_t) || rx_stage_.size_ < sizeof(uint16_t) ||
      (reinterpret_cast<uintptr_t>(tx_stage_.addr_) % HW_CACHE_LINE_SIZE) != 0u ||
      (reinterpret_cast<uintptr_t>(rx_stage_.addr_) % HW_CACHE_LINE_SIZE) != 0u ||
      (tx_stage_.size_ % HW_CACHE_LINE_SIZE) != 0u ||
      (rx_stage_.size_ % HW_CACHE_LINE_SIZE) != 0u ||
      request_irq == nullptr)
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
  base_ = I2C0_BASE + static_cast<uintptr_t>(index) * STRIDE;
  if (SetConfig(config) != ErrorCode::OK)
  {
    base_ = 0u;
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
      Register32(PLIC_CLAIM_COMPLETE) = IRQ0 + index;
      asm volatile("fence iorw, iorw" ::: "memory");
      irq_registered_[index] = true;
    }
  }
  if (!registration_ok)
  {
    SG200XI2C* expected = this;
    (void)instances_[index].compare_exchange_strong(
        expected, nullptr, std::memory_order_acq_rel, std::memory_order_acquire);
    base_ = 0u;
    controller_index_ = 0xFFu;
    controller_claimed_[index].store(false, std::memory_order_release);
  }
}

SG200XI2C::~SG200XI2C()
{
  const uint8_t index = controller_index_;
  if (index >= CONTROLLER_COUNT)
  {
    return;
  }

  faulted_.store(true, std::memory_order_release);
  if (active_.load(std::memory_order_acquire))
  {
    Complete(ErrorCode::FAILED, false, false);
  }
  if (base_ != 0u)
  {
    Register32(base_, REG_DMA_CR) = 0u;
    Register32(base_, REG_INTR_MASK) = 0u;
    (void)Enable(false);
    const uint32_t clear = Register32(base_, REG_CLR_INTR);
    (void)clear;
  }
  asm volatile("fence iorw, iorw" ::: "memory");
  SG200XI2C* expected = this;
  (void)instances_[index].compare_exchange_strong(
      expected, nullptr, std::memory_order_acq_rel, std::memory_order_acquire);
  base_ = 0u;
  controller_index_ = 0xFFu;
  controller_claimed_[index].store(false, std::memory_order_release);
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
  if (base_ == 0u)
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
  const uint32_t standard_high = SclHighCount(input_clock_hz_, STANDARD_HIGH_NS);
  const uint32_t standard_low = SclLowCount(input_clock_hz_, STANDARD_LOW_NS);
  const uint32_t fast_high = SclHighCount(input_clock_hz_, FAST_HIGH_NS);
  const uint32_t fast_low = SclLowCount(input_clock_hz_, FAST_LOW_NS);
  const uint32_t sda_hold = ClockCyclesForNanoseconds(input_clock_hz_, SDA_HOLD_NS);
  const uint32_t sda_setup = ClockCyclesForNanoseconds(input_clock_hz_, SDA_SETUP_NS);
  const uint32_t spike_length =
      ClockCyclesForNanoseconds(input_clock_hz_, SPIKE_SUPPRESSION_NS);
  if (standard_high == 0u || standard_high > UINT16_MAX || standard_low == 0u ||
      standard_low > UINT16_MAX || fast_high == 0u || fast_high > UINT16_MAX ||
      fast_low == 0u || fast_low > UINT16_MAX || sda_hold == 0u ||
      sda_hold > UINT16_MAX || sda_setup < 2u || sda_setup > UINT8_MAX ||
      spike_length == 0u || spike_length > UINT8_MAX)
  {
    active_.store(false, std::memory_order_release);
    return ErrorCode::NOT_SUPPORT;
  }
  ErrorCode result = Enable(false);
  if (result == ErrorCode::OK)
  {
    Register32(base_, REG_CON) = CON_MASTER | CON_RESTART | CON_SLAVE_DISABLE |
                                 (config.clock_speed == 400000u ? CON_FS : CON_SS);
    Register32(base_, REG_SS_H) = standard_high;
    Register32(base_, REG_SS_L) = standard_low;
    Register32(base_, REG_FS_H) = fast_high;
    Register32(base_, REG_FS_L) = fast_low;
    Register32(base_, REG_SDA_HOLD) = sda_hold;
    Register32(base_, REG_SDA_SETUP) = sda_setup;
    Register32(base_, REG_SPKLEN) = spike_length;
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
  if (in_isr)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (base_ == 0u || slave > 0x3FFu || (read.size_ != 0u && read.addr_ == nullptr) ||
      (prefix_size != 0u && prefix == nullptr) ||
      (write_size != 0u && write == nullptr) || (read_op == nullptr) == (write_op == nullptr))
  {
    return ErrorCode::ARG_ERR;
  }
  if (faulted_.load(std::memory_order_acquire))
  {
    return ErrorCode::STATE_ERR;
  }
  Operation<ErrorCode>& requested_operation =
      read_op != nullptr ? *read_op : *write_op;
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
  uint32_t con = Register32(base_, REG_CON);
  Register32(base_, REG_CON) = slave > 0x7Fu ? con | CON_10B : con & ~CON_10B;
  Register32(base_, REG_TAR) = slave | (slave > 0x7Fu ? TAR_10B : 0u);
  Register32(base_, REG_DMA_TDLR) = 0u;
  Register32(base_, REG_DMA_RDLR) = 0u;
  Register32(base_, REG_INTR_MASK) = INTR_ABRT | INTR_STOP;
  const uint32_t clear = Register32(base_, REG_CLR_INTR);
  (void)clear;
  Register32(base_, REG_DMA_CR) = read.size_ ? 3u : 2u;
  result = Enable(true);
  if (result != ErrorCode::OK)
  {
    Complete(result, false, false);
    return result;
  }
  operation_.MarkAsRunning();
  const auto idx = static_cast<uint8_t>((base_ - I2C0_BASE) / STRIDE);
  if (read.size_)
  {
    result = SG200XDMAC::Start(
        rx_channel_, {.memory = reinterpret_cast<uintptr_t>(rx_stage_.addr_),
                      .memory_capacity = rx_stage_.size_,
                      .peripheral = base_ + REG_DATA_CMD,
                      .count = read.size_,
                      .request = static_cast<SG200XDMAC::Request>(
                          static_cast<uint8_t>(SG200XDMAC::Request::I2C0_RX) + idx * 2u),
                      .direction = SG200XDMAC::Direction::PERIPHERAL_TO_MEMORY,
                      .width = SG200XDMAC::Width::HALF_WORD,
                      .mode = SG200XDMAC::Mode::NORMAL,
                      .callback = &DmaRx,
                      .context = this});
  }
  if (result == ErrorCode::OK)
  {
    result = SG200XDMAC::Start(
        tx_channel_, {.memory = reinterpret_cast<uintptr_t>(tx_stage_.addr_),
                      .memory_capacity = tx_stage_.size_,
                      .peripheral = base_ + REG_DATA_CMD,
                      .count = commands,
                      .request = static_cast<SG200XDMAC::Request>(
                          static_cast<uint8_t>(SG200XDMAC::Request::I2C0_TX) + idx * 2u),
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
  if (irq < static_cast<int>(IRQ0) ||
      irq >= static_cast<int>(IRQ0 + CONTROLLER_COUNT))
  {
    return 0;
  }
  SG200XI2C* self = instances_[static_cast<uint8_t>(irq - IRQ0)].load(
      std::memory_order_acquire);
  if (self == nullptr || self->base_ == 0u)
  {
    return 0;
  }
  const uint32_t raw = Register32(self->base_, REG_RAW);
  if (raw & INTR_ABRT)
  {
    const uint32_t x = Register32(self->base_, REG_CLR_ABRT);
    (void)x;
    self->Complete(ErrorCode::NO_RESPONSE, true);
  }
  if (raw & INTR_STOP)
  {
    const uint32_t x = Register32(self->base_, REG_CLR_STOP);
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
  Register32(base_, REG_DMA_CR) = 0u;
  Register32(base_, REG_INTR_MASK) = 0u;
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

#pragma once

#include <atomic>
#include <cstdint>

#include "i2c.hpp"
#include "sg200x_dma.hpp"
#include "sg200x_ll_i2c.h"
#include "sg200x_rcc.hpp"

namespace LibXR
{
/**
 * DMA-only DesignWare APB I2C master for the SG200x C906L.
 *
 * Each I2C operation is one addressed Normal DMA transaction. Circular DMA is
 * intentionally not exposed: it cannot repeat the controller's START/STOP,
 * address, NACK, and arbitration state machine and is not part of STM32I2C's
 * LibXR contract.
 */
class SG200XI2C final : public I2C
{
 public:
  static_assert(std::atomic<SG200XI2C*>::is_always_lock_free,
                "SG200XI2C requires lock-free pointer atomics");
  static_assert(std::atomic<bool>::is_always_lock_free,
                "SG200XI2C requires lock-free boolean atomics");

  using Controller = SG200XRCC::I2cController;

  struct DmaChannels
  {
    uint8_t tx = 6u;
    uint8_t rx = 7u;
  };

  SG200XI2C(Controller controller, RawData tx_command_buffer, RawData rx_buffer,
            Configuration config = {100000u}, DmaChannels dma_channels = {6u, 7u});

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op,
                 bool in_isr = false) override;
  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op,
                  bool in_isr = false) override;
  ErrorCode SetConfig(Configuration config) override;
  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op,
                    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                    bool in_isr = false) override;
  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op,
                     MemAddrLength mem_addr_size = MemAddrLength::BYTE_8,
                     bool in_isr = false) override;

  [[nodiscard]] bool IsValid() const noexcept
  {
    return regs_ != nullptr && !faulted_.load(std::memory_order_acquire);
  }

 private:
  static constexpr uint8_t CONTROLLER_COUNT = I2C_COUNT;
  static constexpr uint32_t IRQ0 = IRQ_I2C0;
  static constexpr uint32_t ENABLE_WAIT_ATTEMPTS = 100000u;
  ErrorCode Enable(bool enabled) const;
  ErrorCode Start(uint16_t slave, const uint8_t* prefix, size_t prefix_size,
                  const uint8_t* write, size_t write_size, RawData read,
                  ReadOperation* read_op, WriteOperation* write_op, bool in_isr);
  void Complete(ErrorCode result, bool in_isr = false, bool notify_operation = true);
  void OnDma(bool rx, ErrorCode result, bool in_isr);
  static void DmaTx(void* context, ErrorCode result, bool in_isr);
  static void DmaRx(void* context, ErrorCode result, bool in_isr);
  static int Interrupt(int irq, void* context);

  I2C_Type* regs_ = nullptr;
  uint8_t controller_index_ = 0xFFu;
  uint32_t input_clock_hz_ = 0u;
  RawData tx_stage_{};
  RawData rx_stage_{};
  std::atomic<bool> active_{false};
  std::atomic<bool> finishing_{false};
  std::atomic<bool> faulted_{false};
  DmaChannels dma_channels_{};
  uint8_t tx_channel_ = 0xFFu, rx_channel_ = 0xFFu;
  bool tx_done_ = false, rx_done_ = false, stop_done_ = false;
  size_t dma_rx_bytes_ = 0u;
  RawData read_target_{};
  Operation<ErrorCode> operation_{};
  AsyncBlockWait block_wait_{};

  static std::atomic<bool> controller_claimed_[CONTROLLER_COUNT];
  static std::atomic<SG200XI2C*> instances_[CONTROLLER_COUNT];
  static bool irq_registered_[CONTROLLER_COUNT];
};
}  // namespace LibXR

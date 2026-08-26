#pragma once

#include <atomic>
#include <cstdint>

#include "i2c.hpp"
#include "sg200x_dma.hpp"

namespace LibXR
{
/** DMA-only DesignWare APB I2C master for the SG200x C906L. */
class SG200XI2C final : public I2C
{
 public:
  enum class Controller : uint8_t
  {
    I2C0,
    I2C1,
    I2C2,
    I2C3,
    I2C4
  };
  static constexpr uintptr_t I2C0_BASE = 0x04000000u;
  static constexpr uint32_t DEFAULT_CLOCK_HZ = 100000000u;

  SG200XI2C(Controller controller, RawData tx_command_buffer, RawData rx_buffer,
            uint32_t input_clock_hz = DEFAULT_CLOCK_HZ, Configuration config = {100000u});

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

  [[nodiscard]] bool IsValid() const noexcept { return base_ != 0u; }

 private:
  static constexpr uintptr_t STRIDE = 0x10000u;
  // C906L's FreeRTOS interrupt map assigns I2C0..I2C4 to 32..36. The Linux
  // PLIC source numbers 49..53 belong to a different interrupt namespace.
  static constexpr uint32_t IRQ0 = 32u;
  static constexpr uint32_t REG_CON = 0x00u, REG_TAR = 0x04u, REG_DATA_CMD = 0x10u;
  static constexpr uint32_t REG_SS_H = 0x14u, REG_SS_L = 0x18u, REG_FS_H = 0x1Cu,
                            REG_FS_L = 0x20u;
  static constexpr uint32_t REG_INTR_MASK = 0x30u, REG_RAW = 0x34u, REG_CLR_INTR = 0x40u;
  static constexpr uint32_t REG_CLR_ABRT = 0x54u, REG_CLR_STOP = 0x60u,
                            REG_ENABLE = 0x6Cu;
  static constexpr uint32_t REG_ENABLE_STATUS = 0x9Cu, REG_DMA_CR = 0x88u,
                            REG_DMA_TDLR = 0x8Cu;
  static constexpr uint32_t REG_DMA_RDLR = 0x90u, REG_SDA_HOLD = 0x7Cu,
                            REG_SDA_SETUP = 0x94u, REG_SPKLEN = 0xA0u;
  static constexpr uint32_t CON_MASTER = 1u, CON_SS = 2u, CON_FS = 4u, CON_10B = 16u,
                            CON_RESTART = 32u, CON_SLAVE_DISABLE = 64u;
  static constexpr uint16_t CMD_READ = 0x100u, CMD_STOP = 0x200u, CMD_RESTART = 0x400u;
  static constexpr uint32_t INTR_ABRT = 0x40u, INTR_STOP = 0x200u;

  ErrorCode Enable(bool enabled) const;
  ErrorCode Start(uint16_t slave, const uint8_t* prefix, size_t prefix_size,
                  const uint8_t* write, size_t write_size, RawData read,
                  ReadOperation* read_op, WriteOperation* write_op, bool in_isr);
  void Complete(ErrorCode result);
  void OnDma(bool rx, ErrorCode result);
  static void DmaTx(void* context, ErrorCode result);
  static void DmaRx(void* context, ErrorCode result);
  static int Interrupt(int irq, void* context);

  uintptr_t base_ = 0u;
  uint32_t input_clock_hz_ = 0u;
  RawData tx_stage_{};
  RawData rx_stage_{};
  std::atomic<bool> active_{false};
  uint8_t tx_channel_ = 0xFFu, rx_channel_ = 0xFFu;
  bool tx_done_ = false, rx_done_ = false, stop_done_ = false;
  RawData read_target_{};
  ReadOperation* read_op_ = nullptr;
  WriteOperation* write_op_ = nullptr;
  AsyncBlockWait block_wait_{};
};
}  // namespace LibXR

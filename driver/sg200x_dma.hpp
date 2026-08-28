#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR
{

/** SG200x eight-channel DesignWare AXI DMA controller. */
class SG200XDMAC final
{
 public:
  enum class Request : uint8_t
  {
    NONE = 0u,
    SPI0_RX = 16u,
    SPI0_TX,
    SPI1_RX,
    SPI1_TX,
    SPI2_RX,
    SPI2_TX,
    SPI3_RX,
    SPI3_TX,
    I2C0_RX = 24u,
    I2C0_TX,
    I2C1_RX,
    I2C1_TX,
    I2C2_RX,
    I2C2_TX,
    I2C3_RX,
    I2C3_TX,
    I2C4_RX,
    I2C4_TX,
  };
  enum class Direction : uint8_t
  {
    MEMORY_TO_MEMORY,
    MEMORY_TO_PERIPHERAL,
    PERIPHERAL_TO_MEMORY,
  };
  enum class Width : uint8_t
  {
    BYTE = 0u,
    HALF_WORD = 1u
  };
  enum class Mode : uint8_t
  {
    NORMAL,
    CIRCULAR,
  };
  // Completion is dispatched from the C906L PLIC handler. The consumer
  // receives whether it is executing in ISR context so it can select the
  // appropriate LibXR completion primitive.
  using Callback = void (*)(void*, ErrorCode, bool in_isr);

  struct Transfer
  {
    uintptr_t memory = 0u;
    // Backing allocation size in bytes. Required when DMA writes to memory so
    // cache maintenance can prove that every affected cache line is owned by
    // this transfer.
    size_t memory_capacity = 0u;
    // For MEMORY_TO_MEMORY this is the destination memory address.
    uintptr_t peripheral = 0u;
    // Backing allocation size for the MEMORY_TO_MEMORY destination.
    size_t peripheral_capacity = 0u;
    size_t count = 0u;
    Request request = Request::NONE;
    Direction direction = Direction::MEMORY_TO_PERIPHERAL;
    Width width = Width::BYTE;
    Mode mode = Mode::NORMAL;
    Callback callback = nullptr;
    void* context = nullptr;
  };

  static constexpr uint8_t CHANNEL_COUNT = 8u;
  // Channels 0-3 remain owned by Linux. The TOP interrupt mux and the Linux
  // DMAEngine driver use the same partition, so neither CPU can allocate or
  // acknowledge the other CPU's channel state.
  static constexpr uint32_t OWNED_CHANNEL_MASK = 0xF0u;
  static constexpr uintptr_t BASE = 0x04330000u;
  // SDMA_INTR_CPU2 in the CV181x C906L interrupt configuration.
  static constexpr uint8_t IRQ = 25u;
  static ErrorCode Acquire(uint8_t& channel);
  static ErrorCode Release(uint8_t channel, bool in_isr = false);
  static ErrorCode Start(uint8_t channel, const Transfer& transfer);
  static ErrorCode Abort(uint8_t channel, bool in_isr = false);

 private:
  static ErrorCode Initialize();
  static int InterruptHandler(int irq, void* argument);
  static void CheckInterrupt(bool in_isr);
  static void CleanForDevice(uintptr_t address, size_t size) noexcept;
  static void PrepareForDeviceWrite(uintptr_t address, size_t size) noexcept;
  static void InvalidateForCpu(uintptr_t address, size_t size) noexcept;
};

}  // namespace LibXR

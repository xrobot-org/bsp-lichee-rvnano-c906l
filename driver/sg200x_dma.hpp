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
  using Callback = void (*)(void*, ErrorCode);

  struct Transfer
  {
    uintptr_t memory = 0u;
    // For MEMORY_TO_MEMORY this is the destination memory address.
    uintptr_t peripheral = 0u;
    size_t count = 0u;
    Request request = Request::I2C0_TX;
    Direction direction = Direction::MEMORY_TO_PERIPHERAL;
    Width width = Width::BYTE;
    Callback callback = nullptr;
    void* context = nullptr;
  };

  static constexpr uint8_t CHANNEL_COUNT = 8u;
  static constexpr uintptr_t BASE = 0x04330000u;
  // C906L's local PLIC assignment is SDMA_INTR_CPU2=25 in the SG200x
  // FreeRTOS interrupt map. Linux's device-tree source 29 is a different
  // interrupt namespace and must not be passed to request_irq() here.
  static constexpr uint32_t IRQ = 25u;

  static ErrorCode Acquire(uint8_t& channel);
  static void Release(uint8_t channel);
  static ErrorCode Start(uint8_t channel, const Transfer& transfer);
  static ErrorCode Abort(uint8_t channel);
  static void CheckInterrupt();

 private:
  static ErrorCode Initialize();
  static void CleanForDevice(uintptr_t address, size_t size) noexcept;
  static void InvalidateForCpu(uintptr_t address, size_t size) noexcept;
  static int InterruptHandler(int irq, void* argument);
};

}  // namespace LibXR

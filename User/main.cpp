#include "main.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "camera_mailbox.hpp"
#include "semaphore.hpp"
#include "sg200x_gpio.hpp"
#include "sg200x_ll_csr.h"
#include "sg200x_ll_pinmux.h"
#include "sg200x_spi.hpp"
#include "timer.hpp"

extern "C"
{
#include "task.h"
}

namespace
{
constexpr uint32_t LED_PERIOD_MS = 1000u;
constexpr size_t FRAME_SIZE = 2048u;
constexpr size_t HEADER_SIZE = 28u;
constexpr size_t MAX_PAYLOAD = FRAME_SIZE - HEADER_SIZE;
constexpr uint16_t MAX_PAYLOAD_U16 = static_cast<uint16_t>(MAX_PAYLOAD);
constexpr uint8_t VERSION = 1u;
constexpr uint8_t TYPE_VIDEO = 1u;
constexpr uint8_t TYPE_METADATA = 2u;
constexpr uint8_t TYPE_POLL = 3u;
constexpr uint8_t TYPE_HELLO = 0x10u;
constexpr uint8_t FLAG_ACCESS_UNIT_END = 1u;
constexpr uint8_t FLAG_NAL_START = 1u << 1u;
constexpr uint8_t FLAG_NAL_END = 1u << 2u;
constexpr uint32_t TRACE_ADDRESS = 0x8FFFF100u;
constexpr uint32_t GPIOA26_MASK = 1u << 26u;
constexpr uint32_t RTCSYS_GPIO18_MASK = 1u << 18u;

// SPI2 is multiplexed onto the dedicated SD1 pads on the SG2002 board:
// SD1_CLK=SCK, SD1_CMD=SDO/MOSI, SD1_D0=SDI/MISO, SD1_D3=CS.
alignas(64) uint8_t spi_rx[FRAME_SIZE]{};
alignas(64) uint8_t spi_tx[FRAME_SIZE]{};
uint32_t transmitted_records = 0u;
uint32_t transmitted_access_units = 0u;
uint32_t failed_access_units = 0u;
uint32_t acknowledgement_errors = 0u;
uint32_t transmitted_payload_bytes = 0u;
uint32_t transmitted_wire_bytes = 0u;
uint32_t acknowledgement_polls_retried = 0u;

void UpdateTraceCounters(volatile uint32_t* trace)
{
  trace[6] = transmitted_records;
  trace[7] = transmitted_access_units;
  trace[8] = failed_access_units;
  trace[9] = acknowledgement_errors;
  trace[10] = transmitted_payload_bytes;
  trace[11] = FRAME_SIZE;
  trace[12] = transmitted_wire_bytes;
  trace[13] = acknowledgement_polls_retried;
}

struct LedState
{
  LibXR::SG200XGPIO* led;
  bool level;
};

void BlinkLed(LedState* state)
{
  if (state == nullptr || state->led == nullptr)
  {
    return;
  }
  state->level = !state->level;
  state->led->Write(state->level);
}

void TraceSpi(uint32_t stage, uint32_t value = 0u)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(TRACE_ADDRESS);
  trace[0] = stage;
  trace[1] = value;
  UpdateTraceCounters(trace);
  sgll_csr_dcache_clean_invalidate_range(TRACE_ADDRESS, 64u);
  sgll_csr_fence_io();
}

uint32_t Get32(const uint8_t* source);

void TraceSpiTransfer(const std::array<uint8_t, FRAME_SIZE>& frame,
                      const std::array<uint8_t, FRAME_SIZE>& response,
                      LibXR::ErrorCode result, uint32_t dma_completions)
{
  auto* const trace = reinterpret_cast<volatile uint32_t*>(TRACE_ADDRESS);
  trace[0] = 0x30u;
  trace[1] = static_cast<uint32_t>(result);
  trace[2] = Get32(frame.data());
  trace[3] = Get32(response.data());
  trace[4] = static_cast<uint32_t>(frame[3]) | (static_cast<uint32_t>(response[3]) << 8u);
  trace[5] = dma_completions;
  UpdateTraceCounters(trace);
  sgll_csr_dcache_clean_invalidate_range(TRACE_ADDRESS, 64u);
  sgll_csr_fence_io();
}

void ConfigureSpi2Pads()
{
  // The AIC8800 reset is active-low and shares the SD1 pad group with SPI2.
  // Keep the chip held in reset for the whole SPI2 ownership period; the
  // Linux SDIO node is disabled, but an older boot stage may have released it.
  sgll_gpio_output_write(GPIO0_REGS, GPIOA26_MASK, false);
  sgll_csr_fence_io();
  sgll_gpio_output_enable(GPIO0_REGS, GPIOA26_MASK, true);
  (void)sgll_pinmux_function_set(PINMUX_EMMC_DAT2_OFFSET,
                                 PINMUX_EMMC_DAT2_GPIOA26_FUNCTION);

  // D7/SD1_D3 is wired to the C5 CS input. The SG2002 hardware CS produces a
  // separate boundary when its short FIFO empties, so GPIO18 must hold the
  // slave selected for the complete DMA record. Set the output latch high
  // before selecting the GPIO function to avoid a spurious transaction.
  sgll_gpio_output_write(RTC_GPIO_REGS, RTCSYS_GPIO18_MASK, true);
  sgll_csr_fence_io();
  sgll_gpio_output_enable(RTC_GPIO_REGS, RTCSYS_GPIO18_MASK, true);
  // GPIO direction state survives a remoteproc restart. Clear stale GPIO
  // output enables on the three alternate-function pads before handing them
  // to SPI2; the SSI supplies SDO/SCK output enables through the pinmux.
  sgll_gpio_output_enable(RTC_GPIO_REGS, PINMUX_SD1_GPIO_PAD_MASK, false);
#ifdef SPI2_HARDWARE_CS
  (void)sgll_pinmux_function_set(PINMUX_SD1_D3_OFFSET, PINMUX_SD1_D3_SPI2_CS_FUNCTION);
#else
  (void)sgll_pinmux_function_set(PINMUX_SD1_D3_OFFSET, PINMUX_SD1_D3_GPIO18_FUNCTION);
#endif

  // Select the dedicated SD1 pad bank rather than the alternate MIPI lane set
  // before changing muxes.
  sgll_pinmux_select_sd1_pad_bank();
#ifdef SPI2_GPIO_BITBANG
  constexpr struct
  {
    uint32_t offset;
    uint32_t function;
  } spi2_pad_config[] = {
      {PINMUX_SD1_D0_OFFSET, PINMUX_SD1_D0_GPIO21_FUNCTION},
      {PINMUX_SD1_CMD_OFFSET, PINMUX_SD1_CMD_GPIO22_FUNCTION},
      {PINMUX_SD1_CLK_OFFSET, PINMUX_SD1_CLK_GPIO23_FUNCTION},
  };
#else
  constexpr struct
  {
    uint32_t offset;
    uint32_t function;
  } spi2_pad_config[] = {
      {PINMUX_SD1_D0_OFFSET, PINMUX_SD1_D0_SPI2_SDI_FUNCTION},
      {PINMUX_SD1_CMD_OFFSET, PINMUX_SD1_CMD_SPI2_SDO_FUNCTION},
      {PINMUX_SD1_CLK_OFFSET, PINMUX_SD1_CLK_SPI2_SCK_FUNCTION},
  };
#endif
  for (const auto& pad : spi2_pad_config)
  {
    (void)sgll_pinmux_function_set(pad.offset, pad.function);
  }
  sgll_csr_fence_io();
}

uint32_t Crc32(const uint8_t* bytes, size_t length)
{
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0u; i < length; ++i)
  {
    crc ^= bytes[i];
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
      crc = (crc >> 1u) ^ ((crc & 1u) != 0u ? 0xEDB88320u : 0u);
    }
  }
  return ~crc;
}

void Put16(uint8_t* destination, uint16_t value)
{
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8u);
}

void Put32(uint8_t* destination, uint32_t value)
{
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8u);
  destination[2] = static_cast<uint8_t>(value >> 16u);
  destination[3] = static_cast<uint8_t>(value >> 24u);
}

uint32_t Get32(const uint8_t* source)
{
  return static_cast<uint32_t>(source[0]) | (static_cast<uint32_t>(source[1]) << 8u) |
         (static_cast<uint32_t>(source[2]) << 16u) |
         (static_cast<uint32_t>(source[3]) << 24u);
}

struct AnnexBStartCode
{
  size_t offset;
  size_t length;
};

bool FindAnnexBStartCode(const uint8_t* bytes, size_t length, size_t offset,
                         AnnexBStartCode& start_code)
{
  if (bytes == nullptr || offset >= length)
  {
    return false;
  }

  for (size_t index = offset; index + 3u <= length; ++index)
  {
    if (bytes[index] != 0u || bytes[index + 1u] != 0u)
    {
      continue;
    }
    if (bytes[index + 2u] == 1u)
    {
      start_code = {.offset = index, .length = 3u};
      return true;
    }
    if (index + 4u <= length && bytes[index + 2u] == 0u && bytes[index + 3u] == 1u)
    {
      start_code = {.offset = index, .length = 4u};
      return true;
    }
  }
  return false;
}

void EncodeFrame(std::array<uint8_t, FRAME_SIZE>& frame, uint8_t type, uint8_t flags,
                 uint32_t sequence, uint32_t frame_id, uint32_t timestamp_ms,
                 const uint8_t* payload, uint16_t payload_len)
{
  frame.fill(0u);
  frame[0] = 'R';
  frame[1] = 'Z';
  frame[2] = VERSION;
  frame[3] = type;
  frame[4] = flags;
  Put32(frame.data() + 6u, sequence);
  Put32(frame.data() + 10u, frame_id);
  Put32(frame.data() + 14u, timestamp_ms);
  Put16(frame.data() + 18u, payload_len);
  if (payload_len != 0u && payload != nullptr)
  {
    std::memcpy(frame.data() + HEADER_SIZE, payload, payload_len);
  }
  Put32(frame.data() + 20u, Crc32(frame.data() + HEADER_SIZE, payload_len));
  Put32(frame.data() + 24u, Crc32(frame.data(), 24u));
}

bool AckValid(const std::array<uint8_t, FRAME_SIZE>& response)
{
  return response[0] == 'O' && response[1] == 'K' && response[2] == VERSION &&
         response[3] == 0u;
}

bool SpiTransfer(LibXR::SG200XSPI& spi, const std::array<uint8_t, FRAME_SIZE>& frame,
                 std::array<uint8_t, FRAME_SIZE>& response)
{
  static LibXR::Semaphore semaphore;
  LibXR::WriteOperation operation(semaphore, 1000u);
  const size_t payload_size =
      static_cast<size_t>(frame[18]) | (static_cast<size_t>(frame[19]) << 8u);
  const size_t record_size = (HEADER_SIZE + payload_size + SGLL_DCACHE_LINE_SIZE - 1u) &
                             ~(size_t{SGLL_DCACHE_LINE_SIZE} - 1u);
  const auto result =
      spi.ReadAndWrite(LibXR::RawData(response.data(), record_size),
                       LibXR::ConstRawData(frame.data(), record_size), operation);
  ++transmitted_records;
  transmitted_wire_bytes += record_size;
  TraceSpiTransfer(frame, response, result, spi.DmaReceiveCount());
  // The four-wire link has no READY pin. Three 1 kHz ticks guarantee at least
  // two full milliseconds for CRC validation and DMA rearming; one tick can
  // expire immediately when this transfer finishes at a scheduler boundary.
  vTaskDelay(pdMS_TO_TICKS(3u));
  return result == LibXR::ErrorCode::OK;
}

bool Negotiate(LibXR::SG200XSPI& spi, std::array<uint8_t, FRAME_SIZE>& frame,
               std::array<uint8_t, FRAME_SIZE>& response)
{
  // The first response contains the boot-time ACK. The following POLL clocks
  // out the HELLO acknowledgement while giving the C5 task a transaction
  // boundary before video starts.
  EncodeFrame(frame, TYPE_HELLO, 0u, 0u, 0u, 0u, nullptr, 0u);
  if (!SpiTransfer(spi, frame, response))
  {
    return false;
  }
  for (uint32_t attempt = 0u; attempt < 20u; ++attempt)
  {
    EncodeFrame(frame, TYPE_POLL, 0u, 0u, 0u, 0u, nullptr, 0u);
    if (SpiTransfer(spi, frame, response) && AckValid(response) &&
        Get32(response.data() + 4u) == 0u)
    {
      std::printf("SPI HELLO acknowledged (accepted=%lu)\n",
                  static_cast<unsigned long>(Get32(response.data() + 8u)));
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10u));
  }
  std::printf("SPI HELLO timeout\n");
  return false;
}

bool SendNalu(LibXR::SG200XSPI& spi, std::array<uint8_t, FRAME_SIZE>& frame,
              std::array<uint8_t, FRAME_SIZE>& response, uint32_t& sequence,
              const CameraMailbox::AccessUnit& access_unit, const uint8_t* nalu,
              size_t length, bool ends_access_unit)
{
  if (nalu == nullptr || length == 0u)
  {
    return false;
  }
  for (size_t offset = 0u; offset < length;)
  {
    const size_t bytes = std::min(MAX_PAYLOAD, length - offset);
    uint8_t flags = offset == 0u ? FLAG_NAL_START : 0u;
    if (offset + bytes == length)
    {
      flags |= FLAG_NAL_END;
      if (ends_access_unit)
      {
        flags |= FLAG_ACCESS_UNIT_END;
      }
    }
    EncodeFrame(frame, TYPE_VIDEO, flags, sequence++, access_unit.frame_id,
                access_unit.timestamp_ms, nalu + offset, static_cast<uint16_t>(bytes));
    if (!SpiTransfer(spi, frame, response) || !AckValid(response))
    {
      ++acknowledgement_errors;
      return false;
    }
    offset += bytes;
  }
  return true;
}

bool SendAccessUnit(LibXR::SG200XSPI& spi, std::array<uint8_t, FRAME_SIZE>& frame,
                    std::array<uint8_t, FRAME_SIZE>& response, uint32_t& sequence,
                    const CameraMailbox::AccessUnit& access_unit)
{
  const uint8_t* const data = access_unit.data;
  AnnexBStartCode current_start{};
  if (!FindAnnexBStartCode(data, access_unit.length, 0u, current_start))
  {
    return SendNalu(spi, frame, response, sequence, access_unit, data, access_unit.length,
                    true);
  }

  size_t start = current_start.offset + current_start.length;
  bool sent_nalu = false;
  for (;;)
  {
    AnnexBStartCode next_start{};
    const bool has_next =
        FindAnnexBStartCode(data, access_unit.length, start, next_start);
    size_t end = has_next ? next_start.offset : access_unit.length;
    while (end > start && data[end - 1u] == 0u)
    {
      --end;
    }

    if (end > start && !SendNalu(spi, frame, response, sequence, access_unit,
                                 data + start, end - start, !has_next))
    {
      return false;
    }
    sent_nalu = sent_nalu || end > start;
    if (!has_next)
    {
      return sent_nalu;
    }
    start = next_start.offset + next_start.length;
  }
}

void SpiTxTask(void* argument)
{
  auto* spi = static_cast<LibXR::SG200XSPI*>(argument);
  if (spi == nullptr)
  {
    vTaskDelete(nullptr);
    return;
  }

  std::array<uint8_t, FRAME_SIZE> frame{};
  std::array<uint8_t, FRAME_SIZE> response{};
  uint32_t sequence = 1u;
  uint32_t frame_id = 0u;
  bool negotiated = false;

  // The C5 starts its SPI slave after a 15-second SoftAP isolation window.
  // Start after that window so the first SSI transaction is captured by an
  // armed DMA slave instead of being discarded during boot.
  vTaskDelay(pdMS_TO_TICKS(20000u));
  for (;;)
  {
    if (!negotiated && !Negotiate(*spi, frame, response))
    {
      vTaskDelay(pdMS_TO_TICKS(500u));
      continue;
    }
    negotiated = true;

    CameraMailbox::AccessUnit access_unit{};
    const auto mailbox_result = CameraMailbox::Acquire(access_unit);
    if (mailbox_result != CameraMailbox::Result::Ready)
    {
      if (mailbox_result == CameraMailbox::Result::InvalidHeader)
      {
        std::printf("camera mailbox header rejected\n");
      }
      else if (mailbox_result == CameraMailbox::Result::InvalidPayload)
      {
        std::printf("camera mailbox payload rejected\n");
      }
      vTaskDelay(pdMS_TO_TICKS(1u));
      continue;
    }

    bool transfer_failed = !SendAccessUnit(*spi, frame, response, sequence, access_unit);

    uint32_t final_sequence = 0u;
    if (!transfer_failed)
    {
      // Metadata payload: version, target-valid flag, optional target fields,
      // then the 16x16 image dimensions. No selected target is intentional.
      uint8_t metadata[34]{};
      metadata[0] = 1u;
      Put16(metadata + 20u, access_unit.width);
      Put16(metadata + 22u, access_unit.height);
      final_sequence = sequence++;
      EncodeFrame(frame, TYPE_METADATA, 0u, final_sequence, access_unit.frame_id,
                  access_unit.timestamp_ms, metadata, sizeof(metadata));
      if (!SpiTransfer(*spi, frame, response) || !AckValid(response))
      {
        transfer_failed = true;
      }
      else if ((frame_id & 0x0Fu) == 0u)
      {
        std::printf("SPI frame %lu acknowledged through %lu\n",
                    static_cast<unsigned long>(frame_id),
                    static_cast<unsigned long>(Get32(response.data() + 4u)));
      }
    }

    if (!transfer_failed)
    {
      EncodeFrame(frame, TYPE_POLL, 0u, 0u, access_unit.frame_id,
                  access_unit.timestamp_ms, nullptr, 0u);
      bool acknowledged = false;
      for (uint32_t attempt = 0u; attempt < 3u; ++attempt)
      {
        if (attempt != 0u) ++acknowledgement_polls_retried;
        if (!SpiTransfer(*spi, frame, response)) break;
        if (AckValid(response) && Get32(response.data() + 4u) == final_sequence)
        {
          acknowledged = true;
          break;
        }
      }
      if (!acknowledged)
      {
        ++acknowledgement_errors;
        transfer_failed = true;
      }
    }

    CameraMailbox::Release();
    if (transfer_failed)
    {
      ++failed_access_units;
      negotiated = false;
      // The bridge publishes one access unit at a time. Do not leave it
      // blocked when SPI is temporarily unavailable; the next key frame will
      // reestablish decoder state after the link is negotiated again.
      TraceSpi(0x21u, access_unit.frame_id);
      vTaskDelay(pdMS_TO_TICKS(100u));
      continue;
    }
    ++transmitted_access_units;
    transmitted_payload_bytes += access_unit.length;
    frame_id = access_unit.frame_id + 1u;
    TraceSpi(0x40u, frame_id);
  }
}
}  // namespace

extern "C" void CreateDefaultTask(void)
{
  // GPIOA_14 is the active-low user LED on LicheeRV Nano.
  static LibXR::SG200XGPIO led(LibXR::SG200XGPIO::Bank::A, 14u);
  const auto result =
      led.SetConfig({LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (result != LibXR::ErrorCode::OK)
  {
    return;
  }
  led.Write(false);

  static LedState state{&led, false};
  const auto timer = LibXR::Timer::CreateTask<LedState*>(BlinkLed, &state, LED_PERIOD_MS);
  if (timer == nullptr)
  {
    return;
  }
  LibXR::Timer::Add(timer);
  LibXR::Timer::Start(timer);

  ConfigureSpi2Pads();
  static LibXR::SG200XSPI spi(
      LibXR::SG200XRCC::SpiController::SPI2, 0u, LibXR::RawData(spi_rx, sizeof(spi_rx)),
      LibXR::RawData(spi_tx, sizeof(spi_tx)),
      {LibXR::SPI::ClockPolarity::LOW, LibXR::SPI::ClockPhase::EDGE_2,
       // 187.5 MHz / (2 * 16) = 5.859375 MHz; both directions use DMA.
       LibXR::SPI::Prescaler::DIV_16, false});
  if (!spi.IsValid())
  {
    TraceSpi(0x02u);
    std::printf("SG200X SPI2 controller initialization failed\n");
    return;
  }
  std::printf("SG200X SPI2 ready at %lu Hz\n",
              static_cast<unsigned long>(spi.ActualBusSpeed()));
  if (xTaskCreate(SpiTxTask, "spi_tx", 4096u, &spi, 3u, nullptr) != pdPASS)
  {
    TraceSpi(0x03u);
    std::printf("failed to create SPI task\n");
  }
}

extern "C" void DefaultTask(void* argument)
{
  BlinkLed(static_cast<LedState*>(argument));
}

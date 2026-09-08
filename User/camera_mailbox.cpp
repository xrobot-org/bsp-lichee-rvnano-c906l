/**
 * @file camera_mailbox.cpp
 * @brief Camera bridge wire-format validation over the SGLL mailbox transport.
 * @see tools/sg2002-camera-bridge/bridge.c.
 */
#include "camera_mailbox.hpp"

#include "sg200x_ll_mbox.h"

namespace
{
constexpr uintptr_t MAILBOX_ADDRESS = 0x8FFD'E000UL;
constexpr size_t MAILBOX_SIZE = 0x2'1000U;
constexpr size_t HEADER_SIZE = 64U;
constexpr uint32_t MAGIC = 0x4D43'5A52U;
constexpr uint32_t VERSION = 1U;
constexpr uint32_t EMPTY = 0U;
constexpr uint32_t READY = 1U;

/**
 * @brief Version 1 camera bridge header; the payload follows these 64 bytes.
 * @note This is an application protocol, separate from SG2002 hardware registers.
 */
struct Header
{
  uint32_t magic;
  uint32_t version;
  uint32_t state;
  uint32_t frame_id;
  uint32_t length;
  uint32_t timestamp_ms;
  uint16_t width;
  uint16_t height;
  uint32_t crc32;
  uint32_t producer_drops;
  uint32_t reserved[7];
};

static_assert(sizeof(Header) == HEADER_SIZE);
static_assert(offsetof(Header, state) == 8U);
static_assert(offsetof(Header, frame_id) == 12U);
static_assert(offsetof(Header, length) == 16U);
static_assert(offsetof(Header, timestamp_ms) == 20U);
static_assert(offsetof(Header, width) == 24U);
static_assert(offsetof(Header, height) == 26U);
static_assert(offsetof(Header, crc32) == 28U);
static_assert(offsetof(Header, producer_drops) == 32U);

constexpr sgll_mbox_config_t MAILBOX = {
    .base = MAILBOX_ADDRESS,
    .size = MAILBOX_SIZE,
    .header_size = HEADER_SIZE,
    .state_offset = offsetof(Header, state),
    .empty_state = EMPTY,
    .ready_state = READY,
};

uint32_t Crc32(const uint8_t* bytes, size_t length)
{
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0U; i < length; ++i)
  {
    crc ^= bytes[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB8'8320U : 0U);
    }
  }
  return ~crc;
}
}  // namespace

CameraMailbox::Result CameraMailbox::Acquire(AccessUnit& access_unit)
{
  Header header{};
  const auto result = sgll_mbox_rx_acquire(&MAILBOX, &header, sizeof(header));
  if (result == SGLL_MBOX_BUSY)
  {
    return Result::Empty;
  }
  if (result != SGLL_MBOX_OK)
  {
    return Result::InvalidHeader;
  }
  if (header.magic != MAGIC || header.version != VERSION || header.length == 0U ||
      header.length > MAILBOX_SIZE - HEADER_SIZE)
  {
    Release();
    return Result::InvalidHeader;
  }

  const uint8_t* payload = nullptr;
  if (sgll_mbox_rx_payload(&MAILBOX, header.length, &payload) != SGLL_MBOX_OK ||
      Crc32(payload, header.length) != header.crc32)
  {
    Release();
    return Result::InvalidPayload;
  }

  access_unit = {.data = payload,
                 .length = header.length,
                 .frame_id = header.frame_id,
                 .timestamp_ms = header.timestamp_ms,
                 .width = header.width,
                 .height = header.height};
  return Result::Ready;
}

void CameraMailbox::Release() { (void)sgll_mbox_rx_release(&MAILBOX); }

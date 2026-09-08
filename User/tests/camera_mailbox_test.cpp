/** @file camera_mailbox_test.cpp @brief Compatibility with the Linux bridge's version 1
 * wire format. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "camera_mailbox.hpp"

#include <sys/mman.h>

#include <cassert>
#include <cstring>

namespace
{
constexpr uintptr_t ADDRESS = 0x8FFDE000U;
constexpr size_t SIZE = 0x21000U;
constexpr size_t HEADER = 64U;
auto* const bytes = reinterpret_cast<uint8_t*>(ADDRESS);

void Put16(size_t offset, uint16_t value)
{
  bytes[offset] = static_cast<uint8_t>(value);
  bytes[offset + 1U] = static_cast<uint8_t>(value >> 8U);
}

void Put32(size_t offset, uint32_t value)
{
  for (unsigned byte = 0; byte < 4U; ++byte)
  {
    bytes[offset + byte] = static_cast<uint8_t>(value >> (8U * byte));
  }
}

uint32_t State()
{
  return static_cast<uint32_t>(bytes[8]) | (static_cast<uint32_t>(bytes[9]) << 8U) |
         (static_cast<uint32_t>(bytes[10]) << 16U) |
         (static_cast<uint32_t>(bytes[11]) << 24U);
}

void Publish()
{
  std::memset(bytes, 0, HEADER);
  Put32(0, 0x4D435A52U);
  Put32(4, 1U);
  Put32(12, 123U);
  Put32(16, 9U);
  Put32(20, 456U);
  Put16(24, 640U);
  Put16(26, 480U);
  Put32(28, 0xCBF43926U);  // Standard CRC-32/ISO-HDLC check value for "123456789".
  Put32(32, 17U);
  std::memcpy(bytes + HEADER, "123456789", 9U);
  Put32(8, 1U);
}

bool Equal(const CameraMailbox::AccessUnit& a, const CameraMailbox::AccessUnit& b)
{
  return a.data == b.data && a.length == b.length && a.frame_id == b.frame_id &&
         a.timestamp_ms == b.timestamp_ms && a.width == b.width && a.height == b.height;
}
}  // namespace

int main()
{
  void* mapping =
      mmap(reinterpret_cast<void*>(ADDRESS), SIZE + 4096U, PROT_READ | PROT_WRITE,
           MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  assert(mapping == bytes);
  assert(mprotect(bytes + SIZE, 4096U, PROT_NONE) == 0);

  CameraMailbox::AccessUnit unit{.data = bytes,
                                 .length = 7U,
                                 .frame_id = 99U,
                                 .timestamp_ms = 0U,
                                 .width = 0U,
                                 .height = 0U};
  const auto untouched = unit;
  assert(CameraMailbox::Acquire(unit) == CameraMailbox::Result::Empty);
  assert(Equal(unit, untouched));
  CameraMailbox::Release();
  assert(State() == 0U);

  Publish();
  assert(CameraMailbox::Acquire(unit) == CameraMailbox::Result::Ready);
  assert(unit.data == bytes + HEADER && unit.length == 9U);
  assert(unit.frame_id == 123U && unit.timestamp_ms == 456U);
  assert(unit.width == 640U && unit.height == 480U);
  assert(std::memcmp(unit.data, "123456789", 9U) == 0);
  assert(State() == 1U);
  uint8_t expected_header[HEADER];
  std::memcpy(expected_header, bytes, HEADER);
  expected_header[8] = 0U;
  CameraMailbox::Release();
  assert(State() == 0U);
  assert(std::memcmp(expected_header, bytes, HEADER) == 0);

  for (unsigned failure = 0; failure < 6U; ++failure)
  {
    Publish();
    unit = untouched;
    switch (failure)
    {
      case 0:
        Put32(0, 0U);
        break;
      case 1:
        Put32(4, 2U);
        break;
      case 2:
        Put32(16, 0U);
        break;
      case 3:
        Put32(16, SIZE - HEADER + 1U);
        break;
      case 4:
        Put32(16, UINT32_MAX);
        break;
      case 5:
        bytes[HEADER] ^= 1U;
        break;
    }
    const auto expected = failure == 5U ? CameraMailbox::Result::InvalidPayload
                                        : CameraMailbox::Result::InvalidHeader;
    assert(CameraMailbox::Acquire(unit) == expected);
    assert(State() == 0U);
    assert(Equal(unit, untouched));
  }

  Publish();
  Put32(8, 2U);
  assert(CameraMailbox::Acquire(unit) == CameraMailbox::Result::Empty);
  assert(State() == 2U);
  Publish();
  assert(CameraMailbox::Acquire(unit) == CameraMailbox::Result::Ready);
  CameraMailbox::Release();
  assert(State() == 0U);
  assert(munmap(mapping, SIZE + 4096U) == 0);
  return 0;
}

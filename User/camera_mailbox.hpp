/**
 * @file camera_mailbox.hpp
 * @brief Receive camera access units from the Linux shared-DDR bridge.
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace CameraMailbox
{

/** @brief View of one encoded frame, valid until Release(). */
struct AccessUnit
{
  const uint8_t* data;
  size_t length;
  uint32_t frame_id;
  uint32_t timestamp_ms;
  uint16_t width;
  uint16_t height;
};

/** @brief Result of polling the bridge's single message slot. */
enum class Result
{
  Empty,
  Ready,
  InvalidHeader,
  InvalidPayload,
};

/**
 * @brief Acquire and validate the next camera access unit without copying its payload.
 * @param access_unit Receives the frame view; unchanged unless Result::Ready is returned.
 * @return Slot state or the validation failure that caused the message to be dropped.
 * @note There must be one consumer. Rejected messages are released automatically.
 */
Result Acquire(AccessUnit& access_unit);

/**
 * @brief Release the acquired frame after all CPU and DMA readers have finished.
 * @note The Linux producer may overwrite the payload as soon as this returns.
 */
void Release();

}  // namespace CameraMailbox

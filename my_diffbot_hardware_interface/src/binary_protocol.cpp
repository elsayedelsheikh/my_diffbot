/*
 * Copyright (c) 2026 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */

#include "my_diffbot_hardware_interface/binary_protocol.hpp"

#include <cstring>

namespace my_diffbot_hardware_interface
{

// ============================================================================
// CRC16-CCITT (poly=0x1021, init=0xFFFF, MSB-first bitwise)
// ============================================================================

uint16_t crc16_ccitt(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// ============================================================================
// BuildFrame
// ============================================================================

std::vector<uint8_t> BuildFrame(uint8_t msg_id, const std::vector<uint8_t> & payload)
{
  const uint8_t payload_len = static_cast<uint8_t>(payload.size());
  std::vector<uint8_t> frame(5 + payload_len);

  frame[0] = 0xAAu;
  frame[1] = msg_id;
  frame[2] = payload_len;

  if (payload_len > 0) {
    std::memcpy(frame.data() + 3, payload.data(), payload_len);
  }

  const uint16_t crc = crc16_ccitt(frame.data(), 3u + payload_len);
  frame[3 + payload_len] = static_cast<uint8_t>(crc & 0xFFu);            // low byte
  frame[4 + payload_len] = static_cast<uint8_t>((crc >> 8) & 0xFFu);     // high byte

  return frame;
}

// ============================================================================
// ExtractFrame
// ============================================================================

static constexpr std::size_t kMaxRxBufferBytes = 1024u;

ExtractResult ExtractFrame(
  std::vector<uint8_t> & buffer,
  uint8_t & out_msg_id,
  std::vector<uint8_t> & out_payload)
{
  // Overflow guard
  if (buffer.size() > kMaxRxBufferBytes) {
    buffer.clear();
    return ExtractResult::OVERFLOW;
  }

  // Scan for start byte
  std::size_t start = buffer.size();
  for (std::size_t i = 0; i < buffer.size(); ++i) {
    if (buffer[i] == 0xAAu) {
      start = i;
      break;
    }
  }

  // Discard any leading garbage before 0xAA
  if (start > 0) {
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(start));
  }

  if (buffer.empty()) {
    return ExtractResult::NO_START_BYTE;
  }

  // Need at least 3 bytes (START_BYTE + MSG_ID + LENGTH)
  if (buffer.size() < 3u) {
    return ExtractResult::INCOMPLETE;
  }

  const uint8_t payload_len = buffer[2];
  const std::size_t frame_sz = 5u + payload_len;

  if (buffer.size() < frame_sz) {
    return ExtractResult::INCOMPLETE;
  }

  // Validate CRC over [START_BYTE | MSG_ID | LENGTH | PAYLOAD]
  const uint16_t calc_crc = crc16_ccitt(buffer.data(), 3u + payload_len);
  const uint16_t recv_crc =
    static_cast<uint16_t>(buffer[3 + payload_len]) |
    (static_cast<uint16_t>(buffer[4 + payload_len]) << 8);

  if (calc_crc != recv_crc) {
    // Discard the leading 0xAA and let the caller re-scan
    buffer.erase(buffer.begin());
    return ExtractResult::CRC_MISMATCH;
  }

  // Extract
  out_msg_id = buffer[1];
  out_payload.assign(buffer.begin() + 3, buffer.begin() + 3 + payload_len);

  // Remove consumed frame
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(frame_sz));

  return ExtractResult::OK;
}

}  // namespace my_diffbot_hardware_interface

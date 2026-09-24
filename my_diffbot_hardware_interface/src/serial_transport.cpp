/*
 * Copyright (c) 2026 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */

#include "my_diffbot_hardware_interface/serial_transport.hpp"

#include <algorithm>
#include <cstring>

namespace my_diffbot_hardware_interface
{

// ============================================================================
// Connection lifecycle
// ============================================================================

bool
SerialTransport::Connect()
{
  if (!configured_) {
    RCLCPP_ERROR(logger_, "Cannot connect: Configure() has not been called");
    return false;
  }
  try {
    if (IsOpen()) {
      return true;
    }
    serial_dev_.Open(port_);
    serial_dev_.SetBaudRate(ConvertBaudRate(baud_rate_));
    serial_dev_.FlushIOBuffers();
    RCLCPP_INFO(logger_, "Connected to serial port: %s", port_.c_str());
    return IsOpen();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to connect to %s: %s", port_.c_str(), e.what());
    return false;
  }
}

void
SerialTransport::Shutdown()
{
  Reset();

  if (IsOpen()) {
    RCLCPP_INFO(logger_, "Disconnecting serial port: %s", port_.c_str());
    serial_dev_.Close();
    return;
  }
  RCLCPP_INFO(logger_, "Serial port already closed: %s", port_.c_str());
}

bool
SerialTransport::IsOpen() const
{
  return serial_dev_.IsOpen();
}

// ============================================================================
// I/O
// ============================================================================

void
SerialTransport::Read()
{
  if (!IsOpen()) {
    return;
  }

  try {
    const size_t available = serial_dev_.GetNumberOfBytesAvailable();
    if (available == 0) {
      return;
    }

    // Read raw bytes into a temporary buffer and append to rx_buffer_
    LibSerial::DataBuffer raw;
    serial_dev_.Read(raw, available, read_timeout_ms_);
    if (raw.empty()) {
      return;
    }
    // Feed the parser in chunks. After the host stops reading for a while (e.g. the
    // integration test menu waiting for input) the backlog is several KB, which would
    // trip ExtractFrame's 1024 B overflow guard in one go and discard pending ACKs.
    constexpr size_t kChunkBytes = 256u;
    for (size_t offset = 0; offset < raw.size(); offset += kChunkBytes) {
      const size_t end = std::min(raw.size(), offset + kChunkBytes);
      rx_buffer_.insert(
        rx_buffer_.end(),
        raw.begin() + static_cast<std::ptrdiff_t>(offset),
        raw.begin() + static_cast<std::ptrdiff_t>(end));
      ExtractFrames();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error reading from board: %s", e.what());
  }
}

void
SerialTransport::ExtractFrames()
{
  // Extract frames in a loop until no more complete frames remain
  while (true) {
    uint8_t msg_id = 0;
    std::vector<uint8_t> payload;

    const ExtractResult result = ExtractFrame(rx_buffer_, msg_id, payload);

    if (result == ExtractResult::OK) {
      ++received_message_count_;
      RCLCPP_DEBUG_THROTTLE(
        logger_, *clock_, 1000,
        "BINARY FRAME #%d [✓] MSG_ID=0x%02X payload_len=%zu",
        received_message_count_, msg_id, payload.size());

      // Intercept ACK frames (0xF0) for WaitForAck()
      if (msg_id == MSG_ID::ACK && payload.size() >= 10u) {
        pending_ack_msg_id_ = payload[8];
        pending_ack_status_ = static_cast<AckStatus>(payload[9]);
        ack_received_ = true;
      }

      OnMessageReceived(msg_id, payload);
      continue;
    }

    if (result == ExtractResult::CRC_MISMATCH) {
      ++crc_error_count_;
      RCLCPP_WARN(
        logger_,
        "CRC mismatch on port %s (total errors: %u) — byte discarded, re-scanning",
        port_.c_str(), crc_error_count_);
      continue;  // Re-scan from the next byte
    }

    if (result == ExtractResult::OVERFLOW) {
      RCLCPP_ERROR(
        logger_,
        "RX buffer overflow on port %s — buffer cleared",
        port_.c_str());
      break;
    }

    // INCOMPLETE or NO_START_BYTE — wait for more data
    break;
  }
}

// ============================================================================
// Lifecycle
// ============================================================================

void
SerialTransport::Configure(const std::string & port, int baud_rate, size_t read_timeout_ms)
{
  if (port.empty()) {
    RCLCPP_ERROR(logger_, "Cannot configure: port string is empty");
    return;
  }
  port_ = port;
  baud_rate_ = baud_rate;
  read_timeout_ms_ = read_timeout_ms;
  configured_ = true;
  RCLCPP_INFO(
    logger_,
    "configured: port=%s, baud=%d, read_timeout=%zu",
    port_.c_str(), baud_rate_, read_timeout_ms_);

  OnConfigure();
}

void
SerialTransport::Reset()
{
  OnReset();
  rx_buffer_.clear();
  received_message_count_ = 0;
  crc_error_count_ = 0;
}

// ============================================================================
// Outbound
// ============================================================================

bool
SerialTransport::WriteCommand(uint8_t msg_id, const std::vector<uint8_t> & payload)
{
  if (!IsOpen()) {
    RCLCPP_ERROR(logger_, "Cannot write: serial port is not open");
    return false;
  }

  const std::vector<uint8_t> frame = BuildFrame(msg_id, payload);

  try {
    LibSerial::DataBuffer buf(frame.begin(), frame.end());
    serial_dev_.Write(buf);
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 1000,
      "SERIAL CMD: MSG_ID=0x%02X payload_len=%zu", msg_id, payload.size());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to send command 0x%02X: %s", msg_id, e.what());
    return false;
  }
}

AckStatus
SerialTransport::WaitForAck(uint8_t expected_msg_id, uint32_t timeout_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    Read();

    // Check if we got an ACK for the expected MSG_ID.
    // ACKs are dispatched through OnMessageReceived(0xF0, payload) by Read().
    // Subclasses store the last ACK; we use a dedicated pending-ack mechanism.
    // Since WaitForAck is called synchronously after WriteCommand, we intercept
    // the ACK via the Read() → OnMessageReceived chain through a callback set
    // per-call below. For the base class, we use a lightweight polling approach:
    // the pending_ack_ members are checked here.
    if (ack_received_ && pending_ack_msg_id_ == expected_msg_id) {
      ack_received_ = false;

      // Special case: Handshake ACK (0x09) encodes firmware version in status byte,
      // not a standard error code.  Decode it and normalise to OK.
      if (expected_msg_id == MSG_ID::HANDSHAKE) {
        firmware_version_ = static_cast<uint8_t>(pending_ack_status_);
        pending_ack_status_ = AckStatus::OK;
      }

      const AckStatus status = pending_ack_status_;

      // Log non-OK statuses
      if (status == AckStatus::INVALID_PARAM) {
        RCLCPP_ERROR(
          logger_,
          "ACK INVALID_PARAM for MSG_ID=0x%02X — rejected parameter value",
          expected_msg_id);
      } else if (status == AckStatus::COMMAND_FAILED) {
        RCLCPP_ERROR(
          logger_,
          "ACK COMMAND_FAILED for MSG_ID=0x%02X — wrong lifecycle state or execution failure",
          expected_msg_id);
      } else if (status == AckStatus::TIMEOUT) {
        const double elapsed =
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - deadline +
            std::chrono::milliseconds(timeout_ms)).count();
        RCLCPP_ERROR(
          logger_,
          "ACK TIMEOUT for MSG_ID=0x%02X — MCU internal timeout (elapsed: %.1f ms)",
          expected_msg_id, elapsed);
      } else if (status == AckStatus::UNKNOWN) {
        RCLCPP_WARN(
          logger_,
          "ACK UNKNOWN for MSG_ID=0x%02X — unrecognized by MCU",
          expected_msg_id);
      }

      return status;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }

  RCLCPP_ERROR(
    logger_,
    "WaitForAck TIMEOUT: no ACK received for MSG_ID=0x%02X within %u ms",
    expected_msg_id, timeout_ms);
  return AckStatus::TIMEOUT;
}

bool
SerialTransport::IsHeartbeatHealthy(uint32_t max_age_ms) const
{
  if (last_heartbeat_time_ == std::chrono::steady_clock::time_point{}) {
    return false;  // No heartbeat received yet
  }
  const auto age = std::chrono::steady_clock::now() - last_heartbeat_time_;
  return std::chrono::duration_cast<std::chrono::milliseconds>(age).count() <=
         static_cast<int64_t>(max_age_ms);
}

}  // namespace my_diffbot_hardware_interface

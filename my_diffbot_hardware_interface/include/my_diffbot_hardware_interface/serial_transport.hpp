/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_TRANSPORT_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_TRANSPORT_HPP_

#include <libserial/SerialPort.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

#include "my_diffbot_hardware_interface/binary_protocol.hpp"

namespace my_diffbot_hardware_interface
{

// ============================================================================
/// @brief Binary serial-transport base: framing, CRC16, buffering.
///
/// Responsibilities:
///   - Open / close the serial port.
///   - Buffer incoming bytes and extract binary frames using ExtractFrame().
///   - Validate CRC16 on every incoming frame.
///   - Call the virtual hook OnMessageReceived() for each valid frame.
///   - Provide WriteCommand() so subclasses can send binary-framed messages.
///   - Provide WaitForAck() for commands requiring an ACK response.
///
/// Frame format:  [0xAA][MSG_ID][LENGTH][PAYLOAD][CRC16-LE]
///
/// Subclasses MUST:
///   - Call Configure() before Connect().
///   - Override OnMessageReceived() to handle their own protocol.
// ============================================================================
class SerialTransport {
public:
  SerialTransport() = default;

  virtual ~SerialTransport() {Shutdown();}

  // ── Helpers ────────────────────────────────────────────────────────────────
  void SetClock(rclcpp::Clock::SharedPtr clock) {clock_ = clock;}

  /// @brief Return true if the last heartbeat arrived within @p max_age_ms milliseconds.
  bool IsHeartbeatHealthy(uint32_t max_age_ms) const;

  /// @brief Firmware version byte from the last Handshake ACK (0x10 → v1.0).
  uint8_t GetFirmwareVersion() const {return firmware_version_;}

  // ── Connection lifecycle ───────────────────────────────────────────────────
  bool Connect();
  void Shutdown();
  bool IsOpen() const;

  // ── I/O ───────────────────────────────────────────────────────────────────

  /// @brief Drain the RX buffer, extract complete binary frames, validate CRC,
  ///        and dispatch valid messages to OnMessageReceived().
  void Read();

  // ── Lifecycle ─────────────────────────────────────────────────────────────

  /// @brief Set transport parameters, then call OnConfigure() hook.
  void Configure(const std::string & port, int baud_rate = 57600, size_t read_timeout_ms = 100U);

  /// @brief Call OnReset() hook, then clear base state.
  void Reset();

protected:
  // ── Virtual extension points ───────────────────────────────────────────────
  virtual void OnConfigure() {}
  virtual void OnReset() {}

  /// @brief Called once per valid binary frame.
  /// @param msg_id   MSG_ID field from the frame.
  /// @param payload  Payload bytes (already extracted, length validated).
  virtual void OnMessageReceived(uint8_t msg_id, const std::vector<uint8_t> & payload)
  {
    (void)msg_id;
    (void)payload;
  }

  // ── Outbound framing ──────────────────────────────────────────────────────

  /// @brief Build a binary frame and write it to the serial port.
  /// @param msg_id  Message type identifier.
  /// @param payload Payload bytes.
  /// @return true if the write succeeded.
  bool WriteCommand(uint8_t msg_id, const std::vector<uint8_t> & payload);

  /// @brief Block until an ACK frame arrives for @p expected_msg_id or timeout.
  /// @param expected_msg_id  The MSG_ID we are waiting to be ACK'd.
  /// @param timeout_ms       Maximum wait time in milliseconds (default 3000).
  /// @return AckStatus from the MCU, or TIMEOUT if no ACK arrived in time.
  AckStatus WaitForAck(uint8_t expected_msg_id, uint32_t timeout_ms = 3000u);

  // ── Logging ───────────────────────────────────────────────────────────────
  rclcpp::Logger logger_ {rclcpp::get_logger("SerialTransport")};
  rclcpp::Clock::SharedPtr clock_ {std::make_shared<rclcpp::Clock>()};

  // ── Heartbeat timestamp (updated by subclass OnMessageReceived on 0x20) ───
  std::chrono::steady_clock::time_point last_heartbeat_time_{};

  // ── Firmware version (decoded from Handshake ACK status byte) ─────────────
  uint8_t firmware_version_{0u};

private:
  /// Parse and dispatch every complete frame in rx_buffer_.
  void ExtractFrames();

  // ── Baud rate conversion ───────────────────────────────────────────────────
  static LibSerial::BaudRate ConvertBaudRate(int baud_rate)
  {
    switch (baud_rate) {
      case 1200:   return LibSerial::BaudRate::BAUD_1200;
      case 1800:   return LibSerial::BaudRate::BAUD_1800;
      case 2400:   return LibSerial::BaudRate::BAUD_2400;
      case 4800:   return LibSerial::BaudRate::BAUD_4800;
      case 9600:   return LibSerial::BaudRate::BAUD_9600;
      case 19200:  return LibSerial::BaudRate::BAUD_19200;
      case 38400:  return LibSerial::BaudRate::BAUD_38400;
      case 57600:  return LibSerial::BaudRate::BAUD_57600;
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      case 230400: return LibSerial::BaudRate::BAUD_230400;
      default:     return LibSerial::BaudRate::BAUD_57600;
    }
  }

  LibSerial::SerialPort serial_dev_;

  // ── Binary receive buffer ─────────────────────────────────────────────────
  std::vector<uint8_t> rx_buffer_;

  // ── Counts ────────────────────────────────────────────────────────────────
  int      received_message_count_{0};
  uint32_t crc_error_count_{0};

  // ── Configuration ─────────────────────────────────────────────────────────
  std::string port_;
  int         baud_rate_{57600};
  size_t      read_timeout_ms_{100U};
  bool        configured_{false};

  // ── Pending ACK (set by Read() when 0xF0 arrives, consumed by WaitForAck) ─
  bool      ack_received_{false};
  uint8_t   pending_ack_msg_id_{0};
  AckStatus pending_ack_status_{AckStatus::UNKNOWN};
};

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_TRANSPORT_HPP_

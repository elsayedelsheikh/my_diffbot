/*
 * Copyright (c) 2026 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__BINARY_PROTOCOL_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__BINARY_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace my_diffbot_hardware_interface
{

// ============================================================================
// Message ID Constants (RoboAuto subset; names match the firmware's messages.hpp)
// ============================================================================

namespace MSG_ID
{

// PC -> MCU
constexpr uint8_t HANDSHAKE = 0x09;
constexpr uint8_t WHEEL_VELOCITY = 0x01;
constexpr uint8_t SET_PID_GAINS = 0x03;
constexpr uint8_t SET_CMD_TIMEOUT = 0x04;
constexpr uint8_t RESET = 0x05;
constexpr uint8_t DEACTIVATE = 0x08;
constexpr uint8_t RGB_LED_SET = 0x36;

// MCU -> PC
constexpr uint8_t WHEEL_FEEDBACK = 0x10;
constexpr uint8_t IMU_DATA = 0x11;
constexpr uint8_t PID_DEBUG = 0x13;
constexpr uint8_t HEARTBEAT = 0x20;
constexpr uint8_t ACK = 0xF0;
}  // namespace MSG_ID

// ============================================================================
// AckStatus Enum
// ============================================================================

enum class AckStatus : uint8_t
{
  OK              = 0x00,
  INVALID_PARAM   = 0x01,
  COMMAND_FAILED  = 0x02,
  TIMEOUT         = 0x03,
  UNKNOWN         = 0xFF,
};

// ============================================================================
// Payload Structs (packed, little-endian fields)
// ============================================================================

#pragma pack(push, 1)

/// Protocol version sent in the Handshake payload: major*1e6 + minor*1e3 + patch.
constexpr uint32_t PROTOCOL_VERSION = 1000000u;  // v1.0.0

struct HandshakePayload        // 0x09 — 12 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint32_t protocol_ver;       // packed semver: major*1e6 + minor*1e3 + patch
};

struct ACKPayload              // 0xF0 — 10 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint8_t  ack_msg_id;
  uint8_t  status_code;
};

struct WheelVelocityPayload    // 0x01 — 16 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  int32_t  left_wheel_mrps;
  int32_t  right_wheel_mrps;
};

struct SetPIDGainsPayload      // 0x03 — 32 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  int32_t kp_left;
  int32_t ki_left;
  int32_t kd_left;
  int32_t kp_right;
  int32_t ki_right;
  int32_t kd_right;
};

struct SetCommandTimeoutPayload  // 0x04 — 10 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint16_t timeout_ms;
};

struct TimestampPayload          // 0x05, 0x08 — 8 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
};

struct RGBLEDSetPayload           // 0x36 — 12 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint8_t  red;
  uint8_t  green;
  uint8_t  blue;
  uint8_t  mode;
};

struct WheelFeedbackPayload     // 0x10 — 32 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  int64_t  left_encoder_ticks;
  int64_t  right_encoder_ticks;
  // Firmware-calculated wheel speed, UNSIGNED magnitude only (no direction
  // bit) — the host recovers the sign from the encoder tick deltas
  // (see RoboAuto::OnMessageReceived).
  uint32_t left_wheel_mrps;
  uint32_t right_wheel_mrps;
};

struct IMUDataPayload            // 0x11 — 33 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  int16_t lin_ax, lin_ay, lin_az;
  int16_t ang_vx, ang_vy, ang_vz;
  int16_t quat_w, quat_x, quat_y, quat_z;
  // Wire order per the agreed 33 B spec: sys, acc, gyro, mag (bytes 28–31).
  uint8_t cal_sys, cal_acc, cal_gyro, cal_mag;
  int8_t temp_c;
};

struct PidDebugPayload           // 0x13 — 52 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  int32_t  left_duty_permille;   ///< PWM duty, ‰ (signed)
  int32_t  right_duty_permille;
  int32_t  left_setpoint_milli;  ///< PID setpoint × 1000
  int32_t  right_setpoint_milli;
  int32_t  left_integral_milli;  ///< PID integral × 1000
  int32_t  right_integral_milli;
  int64_t  left_encoder_ticks;   ///< raw edge count
  int64_t  right_encoder_ticks;
  uint32_t status;               ///< bit0 = left encoder fault, bit1 = right
};

struct HeartbeatPayload          // 0x20 — 17 B
{
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint32_t uptime_sec;
  uint16_t crc_error_count;
  uint16_t timeout_event_count;
  uint8_t  system_status;
};

#pragma pack(pop)

// ── BNO055 raw → SI scales ───────────────────────────────────────────────────
//
// The IMU_DATA fields are the BNO055's own register values, so converting them
// depends entirely on the sensor's UNIT_SEL register (0x3B). These constants
// assume the power-on defaults, which is what the firmware leaves in place:
//   ACC_UNIT=0  -> m/s^2   (100 LSB = 1 m/s^2)
//   GYR_UNIT=0  -> deg/s   ( 16 LSB = 1 deg/s)
//   TEMP_UNIT=0 -> degC    (  1 LSB = 1 degC, hence no temperature constant)
// The quaternion scale is fixed at 2^14 and is UNIT_SEL-independent.
//
// UNIT_SEL is assumed, never read back: if the firmware ever selects rad/s for
// the gyro (900 LSB = 1 rad/s) every angular rate here silently reads ~3.6x too
// small. test_binary_protocol pins these values so such a change fails loudly.
constexpr double kBno055AccScale = 1.0 / 100.0;    ///< raw -> m/s^2
constexpr double kBno055GyroScale =
  (1.0 / 16.0) * 3.14159265358979323846 / 180.0;   ///< raw -> rad/s
constexpr double kBno055QuatScale = 1.0 / 16384.0;  ///< raw -> unitless

// ============================================================================
// RGB LED (led gpio)
// ============================================================================

// Modes on the led_mode gpio. Only off and solid reach the 0x36 firmware:
// blink and alternate are flipped host-side, since the firmware restarts BLINK on every frame.
constexpr int kLedOff = 0;
constexpr int kLedSolid = 1;
constexpr int kLedBlink = 2;
constexpr int kLedAlternate = 3;

struct LedOutput
{
  uint8_t r, g, b, fw_mode;
  bool operator==(const LedOutput & o) const
  {
    return r == o.r && g == o.g && b == o.b && fw_mode == o.fw_mode;
  }
  bool operator!=(const LedOutput & o) const {return !(*this == o);}
};

/// @brief Resolve the led gpio commands at time @p t_sec into one 0x36 frame.
/// Alternate flips between @p color and @p color_alt every @p period_ms, blink between
/// @p color and black; both are sent as SOLID. A period <= 0 holds @p color.
inline std::optional<LedOutput> ResolveLed(
  int mode, uint32_t color, uint32_t color_alt, double period_ms, double t_sec)
{
  if (mode == kLedOff) {
    return LedOutput{0, 0, 0, kLedOff};
  }
  if (mode < kLedOff || mode > kLedAlternate) {
    return std::nullopt;
  }
  if (mode == kLedBlink) {
    color_alt = 0;
  }
  if (mode != kLedSolid && period_ms > 0.0 &&
    static_cast<int64_t>(t_sec * 1000.0 / period_ms) % 2 == 1)
  {
    color = color_alt;
  }
  return LedOutput{
    static_cast<uint8_t>(color >> 16), static_cast<uint8_t>(color >> 8),
    static_cast<uint8_t>(color), kLedSolid};
}

// ============================================================================
// Protocol Functions
// ============================================================================

/// @brief Compute CRC16-CCITT (poly=0x1021, init=0xFFFF, MSB-first bitwise).
/// Input covers START_BYTE through end of PAYLOAD.
uint16_t crc16_ccitt(const uint8_t * data, size_t length);

/// @brief Build a binary frame for transmission.
/// @param msg_id    Message type identifier.
/// @param payload   Payload bytes (may be empty).
/// @return Framed bytes: [0xAA][MSG_ID][LENGTH][PAYLOAD][CRC16-LE]
std::vector<uint8_t> BuildFrame(uint8_t msg_id, const std::vector<uint8_t> & payload);

/// @brief Frame extraction error codes returned by ExtractFrame().
enum class ExtractResult
{
  OK            =  0,   ///< Frame extracted successfully.
  INCOMPLETE    =  1,   ///< Not enough bytes yet — wait for more data.
  CRC_MISMATCH  = -1,   ///< CRC failed — first byte discarded, re-scan.
  NO_START_BYTE = -2,   ///< No 0xAA found — buffer cleared.
  OVERFLOW      = -3,   ///< Buffer exceeded 1024 bytes — buffer cleared.
};

/// @brief Attempt to extract one valid frame from @p buffer.
///
/// On success the frame fields are written to @p out_msg_id and @p out_payload
/// and the consumed bytes are removed from @p buffer.
///
/// On CRC_MISMATCH the first byte (the mismatched 0xAA) is removed from
/// @p buffer so the caller can immediately re-scan.
///
/// @param[in,out] buffer       Accumulation buffer; modified in place.
/// @param[out]    out_msg_id   MSG_ID of the extracted frame.
/// @param[out]    out_payload  Payload bytes of the extracted frame.
/// @return ExtractResult indicating outcome.
ExtractResult ExtractFrame(
  std::vector<uint8_t> & buffer,
  uint8_t & out_msg_id,
  std::vector<uint8_t> & out_payload);

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__BINARY_PROTOCOL_HPP_

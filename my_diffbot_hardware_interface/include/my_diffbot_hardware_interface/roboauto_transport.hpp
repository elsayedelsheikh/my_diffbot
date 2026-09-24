/*
 * Copyright (c) 2026 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__ROBOAUTO_TRANSPORT_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__ROBOAUTO_TRANSPORT_HPP_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

#include "my_diffbot_hardware_interface/serial_transport.hpp"
#include "my_diffbot_hardware_interface/binary_protocol.hpp"

namespace my_diffbot_hardware_interface
{

// ============================================================================
/// @brief Serial driver for the RoboAuto differential-drive base (ESP32-S3).
///
/// The firmware is stateless: every command acts in any state, so the host
/// sends each one without a lifecycle guard.
// ============================================================================
class RoboAuto : public SerialTransport
{
public:
  RoboAuto() = default;

  // ── Commands ───────────────────────────────────────────────────────────────

  /// @brief Send Handshake (0x09) and await ACK (status byte = firmware version).
  AckStatus Handshake();

  /// @brief Send SetPIDGains (0x03) and await ACK. Gains are × 1000.
  AckStatus SetPIDGains(
    int32_t kp_l, int32_t ki_l, int32_t kd_l,
    int32_t kp_r, int32_t ki_r, int32_t kd_r);

  /// @brief Send SetCommandTimeout (0x04) and await ACK.
  AckStatus SetCommandTimeout(uint16_t timeout_ms);

  /// @brief Send Deactivate (0x08) and await ACK.
  AckStatus Deactivate();

  /// @brief Send Reset (0x05) and await ACK.
  AckStatus Reset();

  /// @brief Send WheelVelocity (0x01) — no ACK wait (high-frequency command).
  AckStatus SetWheelVelocity(double left_rad_s, double right_rad_s);

  /// @brief Send RGBLEDSet (0x36) — fire-and-forget, no ACK.
  /// Mode: 0 = off, 1 = solid (the ESP32-S3 onboard LED).
  AckStatus SetRGBLED(uint8_t red, uint8_t green, uint8_t blue, uint8_t mode);

  // ── Data accessors (called from hardware_interface::read()) ────────────────

  /// @brief Snapshot of the last parsed WHEEL_FEEDBACK frame
  struct WheelFeedbackSnapshot
  {
    uint64_t count{0};
    int64_t left_ticks{0};
    int64_t right_ticks{0};
    double left_mrps{0.0};
    double right_mrps{0.0};
  };
  WheelFeedbackSnapshot GetWheelFeedbackSnapshot() const;

  /// @brief Last PID_DEBUG motor PWM duty [‰], signed.
  int32_t GetLeftDutyPermille() const {return left_duty_permille_;}
  int32_t GetRightDutyPermille() const {return right_duty_permille_;}

  /// @brief Snapshot of the last parsed IMU_DATA frame (SI units)
  struct IMUSnapshot
  {
    uint64_t count{0};        ///< frames parsed since startup — staleness check
    double lin_acc[3]{};      ///< m/s²  (x, y, z)
    double ang_vel[3]{};      ///< rad/s (x, y, z)
    double quat[4]{};         ///< unitless (w, x, y, z)
    int cal[4]{};             ///< sys, gyro, acc, mag (0–3)
    int temp_c{};             ///< °C
  };
  IMUSnapshot GetIMUSnapshot() const {return imu_;}

  /// @brief Return true if IMU is healthy (Heartbeat system_status bit 1).
  bool GetImuHealthy() const {return imu_healthy_.load();}

private:
  // ── SerialTransport hooks ──────────────────────────────────────────────────
  void OnConfigure() override;
  void OnMessageReceived(uint8_t msg_id, const std::vector<uint8_t> & payload) override;

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// @brief Get current timestamp as {sec, nsec}.
  static void GetTimestamp(uint32_t & sec, uint32_t & nsec);

  /// @brief Send a timestamp-only command and await its ACK.
  AckStatus SendTimestampCommand(uint8_t msg_id);

  /// @brief Serialize any trivially-copyable struct into a byte vector.
  template<typename T>
  static std::vector<uint8_t> ToBytes(const T & s)
  {
    std::vector<uint8_t> v(sizeof(T));
    std::memcpy(v.data(), &s, sizeof(T));
    return v;
  }

  // ── Parsed feedback ───────────────────────────────────────────────────────
  int64_t left_encoder_ticks_{0};
  int64_t right_encoder_ticks_{0};
  uint64_t wheel_feedback_count_{0};
  // Firmware wheel speed, signed on parse (wire value is unsigned — direction
  // comes from the tick delta; a zero delta keeps the last known direction).
  double left_wheel_fw_mrps_{0.0};
  double right_wheel_fw_mrps_{0.0};
  int left_wheel_dir_{1};
  int right_wheel_dir_{1};
  // PID_DEBUG (0x13) — only the PWM duty is consumed today.
  int32_t left_duty_permille_{0};
  int32_t right_duty_permille_{0};

  IMUSnapshot imu_{};

  // Heartbeat system_status bit1
  std::atomic<bool> imu_healthy_{false};
};

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__ROBOAUTO_TRANSPORT_HPP_

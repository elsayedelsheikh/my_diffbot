/*
 * Copyright (c) 2026 RoboLabs
 *
 * Author: ElSayed ElSheikh
 */

#include "my_diffbot_hardware_interface/roboauto_transport.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace my_diffbot_hardware_interface
{

// ============================================================================
// Internal helpers
// ============================================================================

void RoboAuto::GetTimestamp(uint32_t & sec, uint32_t & nsec)
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(now);
  const auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(now) - secs;
  sec = static_cast<uint32_t>(secs.count());
  nsec = static_cast<uint32_t>(nsecs.count());
}

AckStatus RoboAuto::SendTimestampCommand(uint8_t msg_id)
{
  TimestampPayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);

  if (!WriteCommand(msg_id, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  return WaitForAck(msg_id);
}

void RoboAuto::OnConfigure()
{
  logger_ = rclcpp::get_logger("RoboAuto");
}

// ============================================================================
// Commands
// ============================================================================

AckStatus RoboAuto::Handshake()
{
  HandshakePayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);
  p.protocol_ver = PROTOCOL_VERSION;

  if (!WriteCommand(MSG_ID::HANDSHAKE, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  const AckStatus status = WaitForAck(MSG_ID::HANDSHAKE);
  if (status == AckStatus::OK) {
    RCLCPP_INFO(logger_, "Handshake OK — firmware version 0x%02X", GetFirmwareVersion());
  }
  return status;
}

AckStatus RoboAuto::SetPIDGains(
  int32_t kp_l, int32_t ki_l, int32_t kd_l,
  int32_t kp_r, int32_t ki_r, int32_t kd_r)
{
  SetPIDGainsPayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);
  p.kp_left = kp_l;
  p.ki_left = ki_l;
  p.kd_left = kd_l;
  p.kp_right = kp_r;
  p.ki_right = ki_r;
  p.kd_right = kd_r;

  if (!WriteCommand(MSG_ID::SET_PID_GAINS, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  return WaitForAck(MSG_ID::SET_PID_GAINS);
}

AckStatus RoboAuto::SetCommandTimeout(uint16_t timeout_ms)
{
  SetCommandTimeoutPayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);
  p.timeout_ms = timeout_ms;

  if (!WriteCommand(MSG_ID::SET_CMD_TIMEOUT, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  return WaitForAck(MSG_ID::SET_CMD_TIMEOUT);
}

AckStatus RoboAuto::Deactivate()
{
  return SendTimestampCommand(MSG_ID::DEACTIVATE);
}

AckStatus RoboAuto::Reset()
{
  return SendTimestampCommand(MSG_ID::RESET);
}

AckStatus RoboAuto::SetWheelVelocity(double left_rad_s, double right_rad_s)
{
  WheelVelocityPayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);
  p.left_wheel_mrps = static_cast<int32_t>(left_rad_s * 1000.0 / (2.0 * M_PI));
  p.right_wheel_mrps = static_cast<int32_t>(right_rad_s * 1000.0 / (2.0 * M_PI));

  if (!WriteCommand(MSG_ID::WHEEL_VELOCITY, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  return AckStatus::OK;  // No ACK for high-frequency wheel velocity command
}

AckStatus RoboAuto::SetRGBLED(uint8_t red, uint8_t green, uint8_t blue, uint8_t mode)
{
  RGBLEDSetPayload p{};
  GetTimestamp(p.timestamp_sec, p.timestamp_nsec);
  p.red = red;
  p.green = green;
  p.blue = blue;
  p.mode = mode;

  if (!WriteCommand(MSG_ID::RGB_LED_SET, ToBytes(p))) {
    return AckStatus::COMMAND_FAILED;
  }

  return AckStatus::OK;  // No ACK for the RGB LED
}

// ============================================================================
// Feedback parsing
// ============================================================================

void RoboAuto::OnMessageReceived(uint8_t msg_id, const std::vector<uint8_t> & payload)
{
  switch (msg_id) {
    case MSG_ID::WHEEL_FEEDBACK: {
        if (payload.size() < sizeof(WheelFeedbackPayload)) {
          RCLCPP_WARN(logger_, "WheelFeedback: short payload (%zu bytes)", payload.size());
          return;
        }
        WheelFeedbackPayload p{};
        std::memcpy(&p, payload.data(), sizeof(WheelFeedbackPayload));
        // The firmware reports wheel speed as an UNSIGNED magnitude (uint32
        // mrps, no direction). Recover the sign from the encoder tick delta
        // against the previous frame; a zero delta (stationary, or too slow
        // to move a tick per frame) keeps the last known direction — the
        // magnitude is ~0 there anyway.
        if (p.left_encoder_ticks != left_encoder_ticks_) {
          left_wheel_dir_ = (p.left_encoder_ticks > left_encoder_ticks_) ? 1 : -1;
        }
        if (p.right_encoder_ticks != right_encoder_ticks_) {
          right_wheel_dir_ = (p.right_encoder_ticks > right_encoder_ticks_) ? 1 : -1;
        }
        left_wheel_fw_mrps_ =
          static_cast<double>(left_wheel_dir_) * static_cast<double>(p.left_wheel_mrps);
        right_wheel_fw_mrps_ =
          static_cast<double>(right_wheel_dir_) * static_cast<double>(p.right_wheel_mrps);
        left_encoder_ticks_ = p.left_encoder_ticks;
        right_encoder_ticks_ = p.right_encoder_ticks;
        ++wheel_feedback_count_;
        break;
      }

    case MSG_ID::IMU_DATA: {
        if (payload.size() < sizeof(IMUDataPayload)) {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
            "IMUData: short payload (%zu bytes)", payload.size());
          return;
        }
        IMUDataPayload p{};
        std::memcpy(&p, payload.data(), sizeof(IMUDataPayload));

        // Semantic order sys, gyro, acc, mag — mapped by field name so the
        // snapshot stays correct even if the wire layout ever changes.
        imu_.cal[0] = static_cast<int>(p.cal_sys);
        imu_.cal[1] = static_cast<int>(p.cal_gyro);
        imu_.cal[2] = static_cast<int>(p.cal_acc);
        imu_.cal[3] = static_cast<int>(p.cal_mag);

        imu_.lin_acc[0] = static_cast<double>(p.lin_ax) * kBno055AccScale;
        imu_.lin_acc[1] = static_cast<double>(p.lin_ay) * kBno055AccScale;
        imu_.lin_acc[2] = static_cast<double>(p.lin_az) * kBno055AccScale;

        imu_.ang_vel[0] = static_cast<double>(p.ang_vx) * kBno055GyroScale;
        imu_.ang_vel[1] = static_cast<double>(p.ang_vy) * kBno055GyroScale;
        imu_.ang_vel[2] = static_cast<double>(p.ang_vz) * kBno055GyroScale;

        imu_.quat[0] = static_cast<double>(p.quat_w) * kBno055QuatScale;
        imu_.quat[1] = static_cast<double>(p.quat_x) * kBno055QuatScale;
        imu_.quat[2] = static_cast<double>(p.quat_y) * kBno055QuatScale;
        imu_.quat[3] = static_cast<double>(p.quat_z) * kBno055QuatScale;

        imu_.temp_c = static_cast<int>(p.temp_c);
        ++imu_.count;
        break;
      }

    case MSG_ID::PID_DEBUG: {
        if (payload.size() < sizeof(PidDebugPayload)) {
          RCLCPP_WARN(logger_, "PidDebug: short payload (%zu bytes)", payload.size());
          return;
        }
        PidDebugPayload p{};
        std::memcpy(&p, payload.data(), sizeof(PidDebugPayload));
        left_duty_permille_ = p.left_duty_permille;
        right_duty_permille_ = p.right_duty_permille;
        break;
      }

    case MSG_ID::HEARTBEAT: {
        if (payload.size() < sizeof(HeartbeatPayload)) {
          RCLCPP_WARN(logger_, "Heartbeat: short payload (%zu bytes)", payload.size());
          return;
        }
        HeartbeatPayload p{};
        std::memcpy(&p, payload.data(), sizeof(HeartbeatPayload));
        last_heartbeat_time_ = std::chrono::steady_clock::now();
        imu_healthy_.store((p.system_status & 0x02u) != 0u);
        break;
      }

    default:
      // ACKs are consumed by the base class; sonar zeros and VERSION_INFO are unused.
      break;
  }
}

// ============================================================================
// Data accessors
// ============================================================================

RoboAuto::WheelFeedbackSnapshot RoboAuto::GetWheelFeedbackSnapshot() const
{
  WheelFeedbackSnapshot s;
  s.count = wheel_feedback_count_;
  s.left_ticks = left_encoder_ticks_;
  s.right_ticks = right_encoder_ticks_;
  s.left_mrps = left_wheel_fw_mrps_;
  s.right_mrps = right_wheel_fw_mrps_;
  return s;
}

}  // namespace my_diffbot_hardware_interface

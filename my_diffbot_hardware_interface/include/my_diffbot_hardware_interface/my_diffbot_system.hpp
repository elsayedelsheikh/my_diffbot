/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: Sayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_

#include <array>
#include <cstdint>
#include <optional>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "my_diffbot_hardware_interface/binary_protocol.hpp"
#include "my_diffbot_hardware_interface/roboauto_transport.hpp"

namespace my_diffbot_hardware_interface
{

class My_diffbotSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(My_diffbotSystemHardware)

  // LifecycleNodeInterface methods
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // SystemInterface methods
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void ReadImu(bool publish_diag, diagnostic_msgs::msg::DiagnosticArray & diag_array);
  void ReadWheels(const rclcpp::Duration & period);
  void WriteTuning();
  void WriteLed(const rclcpp::Time & time);

  RoboAuto roboauto_;

  // ── Config (read in on_init, applied in on_activate) ────────────────────────
  // Encoder ticks per wheel revolution; used by read() to convert ticks → rad.
  double cpr_ = 1.0;
  // PID gains (× 1000) forwarded to the MCU at activation via SetPIDGains().
  int32_t kp_l_ = 1000, ki_l_ = 3000, kd_l_ = 0;
  int32_t kp_r_ = 1000, ki_r_ = 3000, kd_r_ = 0;
  // Motor command watchdog timeout (ms) forwarded via SetCommandTimeout().
  uint16_t cmd_timeout_ms_ = 250;
  // Last values sent to the MCU via the roboauto_pid / roboauto_watchdog gpio
  // command interfaces — GpioCommandController re-writes its last message every
  // cycle, so write() only touches the serial port when a value changes.
  // Order: kp_l, ki_l, kd_l, kp_r, ki_r, kd_r.
  std::array<double, 6> sent_pid_gains_ {};
  double sent_cmd_timeout_ms_ = 0.0;

  // ── Runtime state ─────────────────────────────────────────────────────────────
  rclcpp::Time last_active_time_;
  // Ticks of the previous read(); seeded from the first WHEEL_FEEDBACK because
  // the firmware keeps counting across host reconnects.
  bool ticks_init_ = false;
  int64_t left_encoder_prev_ = 0;
  int64_t right_encoder_prev_ = 0;
  // Measured wheel speed: differentiate ticks over a >=40 ms host window
  // (per-cycle diff is staircase-noisy and aliases against the ~50 Hz
  // WHEEL_FEEDBACK stream), then EMA-filter.
  int64_t meas_left_prev_ = 0;
  int64_t meas_right_prev_ = 0;
  double meas_window_sec_ = 0.0;
  std::array<double, 2> meas_rad_ {};   // EMA-filtered speed [rad/s] (L, R)

  // Last 0x36 frame sent; resent on change and at 1 Hz since it has no ACK.
  std::optional<LedOutput> sent_led_;
  rclcpp::Time last_led_send_;

  // ── Introspection mirrors (registered via REGISTER_ROS2_CONTROL_INTROSPECTION) ─
  // Wheel target/measured/firmware velocity in mrps (the SetWheelVelocity wire
  // unit) and PWM duty in ‰, on ~/introspection_data/* for PID tuning plots.
  double left_wheel_target_velocity_ = 0.0;
  double left_wheel_measured_velocity_ = 0.0;
  double right_wheel_target_velocity_ = 0.0;
  double right_wheel_measured_velocity_ = 0.0;
  double left_wheel_firmware_velocity_ = 0.0;
  double right_wheel_firmware_velocity_ = 0.0;
  double left_wheel_pwm_ = 0.0;
  double right_wheel_pwm_ = 0.0;

  // ── IMU diagnostics (cal status + temperature → /diagnostics) ─────────────────
  // Private node used only as a publisher factory — no executor/spin needed for
  // publishing. Created in on_configure, destroyed in on_cleanup.
  rclcpp::Node::SharedPtr diag_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Time last_diag_pub_;
  // Staleness detection: IMU frame counter last seen and when it last changed.
  uint64_t last_imu_count_ = 0;
  rclcpp::Time last_imu_count_change_;
};

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_

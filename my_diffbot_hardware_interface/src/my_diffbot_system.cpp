/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: Sayed ElSheikh
 */
#include "my_diffbot_hardware_interface/my_diffbot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/introspection.hpp>
#include <rclcpp/logging.hpp>

namespace my_diffbot_hardware_interface
{

// UNREGISTER_ROS2_CONTROL_INTROSPECTION expands to an unqualified
// DEFAULT_REGISTRY_KEY (unlike the REGISTER_ macro, which qualifies it) —
// pull it into scope so unregistration compiles outside hardware_interface::.
using hardware_interface::DEFAULT_REGISTRY_KEY;

namespace
{

// rad/s → mrps (milli-rev/s), the SetWheelVelocity wire unit; the introspection
// mirrors use it too.
constexpr double kRadToMrps = 1000.0 / (2.0 * M_PI);

constexpr std::array<const char *, 6> kGainOrder =
{"kp_l", "ki_l", "kd_l", "kp_r", "ki_r", "kd_r"};

// The ESP32 can still be booting (USB re-enumeration) when the controller
// manager starts — retry before failing.
bool HandshakeWithRetry(RoboAuto & board, const rclcpp::Logger & logger)
{
  constexpr int kMaxAttempts = 10;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    if (board.Handshake() == AckStatus::OK) {
      return true;
    }
    RCLCPP_WARN(logger, "RoboAuto: handshake attempt %d/%d failed", attempt, kMaxAttempts);
    if (attempt < kMaxAttempts) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  RCLCPP_ERROR(logger, "RoboAuto: handshake failed after %d attempts!", kMaxAttempts);
  return false;
}

diagnostic_msgs::msg::KeyValue MakeKeyValue(const char * key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

}  // namespace

// ── on_init ──────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto param_int = [this](const std::string & key, int fallback) -> int {
      auto it = info_.hardware_parameters.find(key);
      return (it != info_.hardware_parameters.end()) ? std::stoi(it->second) : fallback;
    };

  const std::string serial_port = info_.hardware_parameters["serial_port"];
  const int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  const int timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cpr_ = static_cast<double>(std::stoi(info_.hardware_parameters["encoder_counts_per_revolution"]));
  RCLCPP_INFO(get_logger(), "Encoder CPR: %.0f", cpr_);

  cmd_timeout_ms_ = static_cast<uint16_t>(param_int("cmd_timeout_ms", 250));
  kp_l_ = param_int("kp_l", 1000);
  ki_l_ = param_int("ki_l", 3000);
  kd_l_ = param_int("kd_l", 0);
  kp_r_ = param_int("kp_r", 1000);
  ki_r_ = param_int("ki_r", 3000);
  kd_r_ = param_int("kd_r", 0);

  // Each wheel joint has one velocity command and position + velocity states.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
      hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Configure the serial driver (does NOT open the port)
  roboauto_.SetClock(get_clock());
  roboauto_.Configure(serial_port, baud_rate, timeout_ms);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Configuring %s ...", get_name().c_str());

  // reset values always when configuring hardware
  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }
  for (const auto &[name, descr] : gpio_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto &[name, descr] : gpio_command_interfaces_) {
    set_command(name, 0.0);
  }
  for (const auto &[name, descr] : sensor_state_interfaces_) {
    set_state(name, 0.0);
  }

  // Seed the tuning gpio commands with the on_init parameter values so the
  // blanket zeroing above doesn't register as a "change to 0" once active.
  const std::array<std::pair<const char *, double>, 7> tuning_seeds = {{
    {"roboauto_pid/kp_l", static_cast<double>(kp_l_)},
    {"roboauto_pid/ki_l", static_cast<double>(ki_l_)},
    {"roboauto_pid/kd_l", static_cast<double>(kd_l_)},
    {"roboauto_pid/kp_r", static_cast<double>(kp_r_)},
    {"roboauto_pid/ki_r", static_cast<double>(ki_r_)},
    {"roboauto_pid/kd_r", static_cast<double>(kd_r_)},
    {"roboauto_watchdog/cmd_timeout_ms", static_cast<double>(cmd_timeout_ms_)},
  }};
  for (const auto & [name, value] : tuning_seeds) {
    if (gpio_command_interfaces_.count(name) != 0) {
      set_command(name, value);
    }
  }

  // A hardware component has no node of its own, so create a private one purely
  // as a publisher factory; publish() needs no executor/spin.
  diag_node_ = std::make_shared<rclcpp::Node>("my_diffbot_hw_diagnostics");
  diag_pub_ = diag_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);

  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "left_wheel.target_velocity", &left_wheel_target_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "left_wheel.measured_velocity", &left_wheel_measured_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "right_wheel.target_velocity", &right_wheel_target_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "right_wheel.measured_velocity", &right_wheel_measured_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "left_wheel.firmware_velocity", &left_wheel_firmware_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION(
    "right_wheel.firmware_velocity", &right_wheel_firmware_velocity_);
  REGISTER_ROS2_CONTROL_INTROSPECTION("left_wheel.pwm", &left_wheel_pwm_);
  REGISTER_ROS2_CONTROL_INTROSPECTION("right_wheel.pwm", &right_wheel_pwm_);

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Activating %s ...", get_name().c_str());

  // command and state should be equal when starting
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }

  if (!roboauto_.Connect()) {
    RCLCPP_ERROR(get_logger(), "RoboAuto: serial port failed to open!");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (!HandshakeWithRetry(roboauto_, get_logger())) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  const rclcpp::Time now = get_clock()->now();
  last_active_time_ = now;
  last_imu_count_change_ = now;
  last_diag_pub_ = now;
  last_led_send_ = now;
  last_imu_count_ = 0;
  ticks_init_ = false;
  meas_rad_ = {0.0, 0.0};
  sent_led_.reset();

  // Non-fatal on a missing ACK: a flaky ACK shouldn't kill bringup, and the
  // roboauto_tuning_controller gpios can always re-send.
  RCLCPP_INFO(get_logger(),
    "RoboAuto: sending PID gains L(%d, %d, %d) R(%d, %d, %d)",
    kp_l_, ki_l_, kd_l_, kp_r_, ki_r_, kd_r_);
  if (roboauto_.SetPIDGains(kp_l_, ki_l_, kd_l_, kp_r_, ki_r_, kd_r_) != AckStatus::OK) {
    RCLCPP_ERROR(get_logger(), "RoboAuto: SetPIDGains not acknowledged!");
  }
  RCLCPP_INFO(get_logger(), "RoboAuto: sending command timeout %u ms", cmd_timeout_ms_);
  if (roboauto_.SetCommandTimeout(cmd_timeout_ms_) != AckStatus::OK) {
    RCLCPP_ERROR(get_logger(), "RoboAuto: SetCommandTimeout not acknowledged!");
  }
  sent_pid_gains_ = {
    static_cast<double>(kp_l_), static_cast<double>(ki_l_),
    static_cast<double>(kd_l_), static_cast<double>(kp_r_),
    static_cast<double>(ki_r_), static_cast<double>(kd_r_)};
  sent_cmd_timeout_ms_ = static_cast<double>(cmd_timeout_ms_);

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s ...", get_name().c_str());

  for (const auto &[name, descr] : gpio_command_interfaces_) {
    set_command(name, 0.0);
  }

  roboauto_.SetWheelVelocity(0.0, 0.0);
  roboauto_.Deactivate();
  roboauto_.Reset();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s ...", get_name().c_str());

  UNREGISTER_ROS2_CONTROL_INTROSPECTION("left_wheel.target_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("left_wheel.measured_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("right_wheel.target_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("right_wheel.measured_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("left_wheel.firmware_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("right_wheel.firmware_velocity");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("left_wheel.pwm");
  UNREGISTER_ROS2_CONTROL_INTROSPECTION("right_wheel.pwm");

  diag_pub_.reset();
  diag_node_.reset();
  roboauto_.Shutdown();

  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type
My_diffbotSystemHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  if (!roboauto_.IsOpen()) {
    RCLCPP_ERROR(get_logger(), "Can't connect to RoboAuto; serial port is not open!");
    return hardware_interface::return_type::ERROR;
  }

  // Drain the RX buffer and parse any new frames.
  roboauto_.Read();

  // The led state interfaces mirror their commands.
  for (const auto & [name, descr] : gpio_command_interfaces_) {
    if (descr.get_prefix_name() == "led") {
      set_state(name, get_command(name));
    }
  }

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  const bool publish_diag = (get_clock()->now() - last_diag_pub_).seconds() >= 1.0;
  if (publish_diag) {
    last_diag_pub_ = get_clock()->now();
  }

  ReadImu(publish_diag, diag_array);
  ReadWheels(period);

  if (publish_diag && !diag_array.status.empty()) {
    diag_array.header.stamp = diag_node_->get_clock()->now();
    diag_pub_->publish(diag_array);
  }

  return hardware_interface::return_type::OK;
}

void My_diffbotSystemHardware::ReadImu(
  bool publish_diag, diagnostic_msgs::msg::DiagnosticArray & diag_array)
{
  // Feed the bno055_imu sensor state interfaces (consumed by
  // imu_sensor_broadcaster → /imu/data → EKF). Quat order in the snapshot is w,x,y,z.
  const auto imu = roboauto_.GetIMUSnapshot();
  for (const auto &[name, descr] : sensor_state_interfaces_) {
    if (descr.get_prefix_name() != "bno055_imu") {
      continue;
    }
    const std::string & iface = descr.get_interface_name();
    if (iface == "orientation.w") {
      set_state(name, imu.quat[0]);
    } else if (iface == "orientation.x") {
      set_state(name, imu.quat[1]);
    } else if (iface == "orientation.y") {
      set_state(name, imu.quat[2]);
    } else if (iface == "orientation.z") {
      set_state(name, imu.quat[3]);
    } else if (iface == "angular_velocity.x") {
      set_state(name, imu.ang_vel[0]);
    } else if (iface == "angular_velocity.y") {
      set_state(name, imu.ang_vel[1]);
    } else if (iface == "angular_velocity.z") {
      set_state(name, imu.ang_vel[2]);
    } else if (iface == "linear_acceleration.x") {
      set_state(name, imu.lin_acc[0]);
    } else if (iface == "linear_acceleration.y") {
      set_state(name, imu.lin_acc[1]);
    } else if (iface == "linear_acceleration.z") {
      set_state(name, imu.lin_acc[2]);
    }
  }

  if (imu.count != last_imu_count_) {
    last_imu_count_ = imu.count;
    last_imu_count_change_ = get_clock()->now();
  }
  if (!publish_diag) {
    return;
  }

  using diagnostic_msgs::msg::DiagnosticStatus;
  const bool stale = (get_clock()->now() - last_imu_count_change_).seconds() > 0.5;
  DiagnosticStatus status;
  status.name = "my_diffbot/imu: BNO055";
  status.hardware_id = "roboauto";
  if (!roboauto_.GetImuHealthy() || stale) {
    status.level = DiagnosticStatus::ERROR;
    status.message = stale ? "IMU data stale (no frames > 0.5 s)" : "MCU reports IMU unhealthy";
  } else if (imu.cal[0] >= 2) {
    status.level = DiagnosticStatus::OK;
    status.message = "IMU OK";
  } else {
    status.level = DiagnosticStatus::WARN;
    status.message = "IMU fusion calibration low (move robot / figure-8)";
  }
  const std::array<std::pair<const char *, std::string>, 6> kvs = {{
    {"cal_sys", std::to_string(imu.cal[0])},
    {"cal_gyro", std::to_string(imu.cal[1])},
    {"cal_acc", std::to_string(imu.cal[2])},
    {"cal_mag", std::to_string(imu.cal[3])},
    {"temperature_c", std::to_string(imu.temp_c)},
    {"frame_count", std::to_string(imu.count)},
  }};
  for (const auto & [key, value] : kvs) {
    status.values.push_back(MakeKeyValue(key, value));
  }
  diag_array.status.push_back(status);
}

void My_diffbotSystemHardware::ReadWheels(const rclcpp::Duration & period)
{
  const auto feedback = roboauto_.GetWheelFeedbackSnapshot();

  // Introspection only: firmware speed (signed by the transport) and PWM duty.
  left_wheel_firmware_velocity_ = feedback.left_mrps;
  right_wheel_firmware_velocity_ = feedback.right_mrps;
  left_wheel_pwm_ = roboauto_.GetLeftDutyPermille();
  right_wheel_pwm_ = roboauto_.GetRightDutyPermille();

  // No WHEEL_FEEDBACK parsed yet.
  if (feedback.count == 0u) {
    return;
  }
  // The ticks are cumulative on the MCU, so the first frame after (re)activation
  // only seeds the previous values; using it as a delta would jump the odometry.
  if (!ticks_init_) {
    left_encoder_prev_ = feedback.left_ticks;
    right_encoder_prev_ = feedback.right_ticks;
    meas_left_prev_ = feedback.left_ticks;
    meas_right_prev_ = feedback.right_ticks;
    meas_window_sec_ = 0.0;
    ticks_init_ = true;
    return;
  }

  const int64_t left_delta = feedback.left_ticks - left_encoder_prev_;
  const int64_t right_delta = feedback.right_ticks - right_encoder_prev_;
  left_encoder_prev_ = feedback.left_ticks;
  right_encoder_prev_ = feedback.right_ticks;

  // Differentiate over a >=40 ms window, then EMA-filter: ticks only change when
  // a WHEEL_FEEDBACK frame (~50 Hz) parses, so a per-cycle diff aliases
  // (0 on frame-less cycles, 2x on double-frame cycles).
  constexpr double kMeasWindowSec = 0.04;
  constexpr double kMeasEmaAlpha = 0.4;
  meas_window_sec_ += period.seconds();
  if (meas_window_sec_ >= kMeasWindowSec) {
    const double scale = 2.0 * M_PI / (cpr_ * meas_window_sec_);
    meas_rad_[0] += kMeasEmaAlpha *
      (static_cast<double>(feedback.left_ticks - meas_left_prev_) * scale - meas_rad_[0]);
    meas_rad_[1] += kMeasEmaAlpha *
      (static_cast<double>(feedback.right_ticks - meas_right_prev_) * scale - meas_rad_[1]);
    meas_left_prev_ = feedback.left_ticks;
    meas_right_prev_ = feedback.right_ticks;
    meas_window_sec_ = 0.0;
  }

  for (const auto &[name, descr] : joint_state_interfaces_) {
    if (descr.get_interface_name() != hardware_interface::HW_IF_POSITION) {
      continue;
    }
    const std::string prefix = descr.get_prefix_name();
    const bool left = prefix.find("left") != std::string::npos;
    const int64_t encoder_delta = left ? left_delta : right_delta;
    const double velocity = left ? meas_rad_[0] : meas_rad_[1];

    // One revolution = cpr_ ticks = 2π rad.
    set_state(name, get_state(name) + static_cast<double>(encoder_delta) * 2.0 * M_PI / cpr_);
    set_state(prefix + "/" + hardware_interface::HW_IF_VELOCITY, velocity);

    if (left) {
      left_wheel_measured_velocity_ = velocity * kRadToMrps;
    } else {
      right_wheel_measured_velocity_ = velocity * kRadToMrps;
    }
  }
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "States: L_ticks=%ld , R_ticks=%ld ", feedback.left_ticks, feedback.right_ticks);
}

// ── write ────────────────────────────────────────────────────────────────────
hardware_interface::return_type
My_diffbotSystemHardware::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & /* period */)
{
  if (!roboauto_.IsOpen()) {
    RCLCPP_ERROR(get_logger(), "Can't connect to RoboAuto; serial port is not open!");
    return hardware_interface::return_type::ERROR;
  }

  WriteTuning();
  WriteLed(time);

  double left_cmd = 0.0;
  double right_cmd = 0.0;
  for (const auto &[name, descr] : joint_command_interfaces_) {
    const std::string prefix = descr.get_prefix_name();
    const double cmd = get_command(name);
    if (prefix.find("left") != std::string::npos) {
      left_cmd = cmd;
    } else if (prefix.find("right") != std::string::npos) {
      right_cmd = cmd;
    } else {
      RCLCPP_WARN(get_logger(), "Unknown joint '%s' when mapping command to motor",
        prefix.c_str());
    }
  }

  left_wheel_target_velocity_ = left_cmd * kRadToMrps;
  right_wheel_target_velocity_ = right_cmd * kRadToMrps;

  // Once fully stopped, keep commanding zero for a full second to confirm a
  // sustained stop (not a transient zero between e.g. forward→backward), then
  // go quiet — the board's own command timeout holds the wheels at zero.
  const bool stopped = (left_cmd == 0.0 && right_cmd == 0.0);
  if (!stopped) {
    last_active_time_ = get_clock()->now();
  }
  const bool in_stop_grace = (get_clock()->now() - last_active_time_).seconds() < 1.0;

  if (!stopped || in_stop_grace) {
    if (roboauto_.SetWheelVelocity(left_cmd, right_cmd) != AckStatus::OK) {
      RCLCPP_ERROR(get_logger(),
          "Failed to send Motor commands: left=%.3f rad/s  right=%.3f rad/s",
          left_cmd, right_cmd);
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Motor commands: left=%.3f rad/s  right=%.3f rad/s", left_cmd, right_cmd);
  return hardware_interface::return_type::OK;
}

void My_diffbotSystemHardware::WriteTuning()
{
  // GpioCommandController re-writes its last message every cycle, so only touch
  // the serial port when a value actually changes. Trackers update after the
  // send attempt regardless of ACK — a bad ACK must not retry at the full rate.
  std::array<double, 6> pid_cmd = sent_pid_gains_;
  double timeout_cmd = sent_cmd_timeout_ms_;
  for (const auto & [name, descr] : gpio_command_interfaces_) {
    const std::string prefix = descr.get_prefix_name();
    const std::string iface = descr.get_interface_name();
    if (prefix == "roboauto_pid") {
      for (size_t i = 0; i < kGainOrder.size(); ++i) {
        if (iface == kGainOrder[i]) {
          pid_cmd[i] = get_command(name);
        }
      }
    } else if (prefix == "roboauto_watchdog" && iface == "cmd_timeout_ms") {
      timeout_cmd = get_command(name);
    }
  }

  if (pid_cmd != sent_pid_gains_) {
    std::array<int32_t, 6> gains {};
    for (size_t i = 0; i < gains.size(); ++i) {
      gains[i] = static_cast<int32_t>(pid_cmd[i]);
    }
    RCLCPP_INFO(get_logger(), "RoboAuto: re-tuning PID gains L(%d, %d, %d) R(%d, %d, %d)",
      gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
    if (roboauto_.SetPIDGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]) !=
      AckStatus::OK)
    {
      RCLCPP_ERROR(get_logger(), "RoboAuto: SetPIDGains not acknowledged!");
    }
    sent_pid_gains_ = pid_cmd;
  }

  if (timeout_cmd != sent_cmd_timeout_ms_) {
    const auto timeout_ms = static_cast<uint16_t>(std::max(timeout_cmd, 0.0));
    RCLCPP_INFO(get_logger(), "RoboAuto: setting command timeout %u ms", timeout_ms);
    if (roboauto_.SetCommandTimeout(timeout_ms) != AckStatus::OK) {
      RCLCPP_ERROR(get_logger(), "RoboAuto: SetCommandTimeout not acknowledged!");
    }
    sent_cmd_timeout_ms_ = timeout_cmd;
  }
}

void My_diffbotSystemHardware::WriteLed(const rclcpp::Time & time)
{
  if (gpio_command_interfaces_.count("led/led_mode") == 0u) {
    return;
  }
  const int led_mode = static_cast<int>(get_command("led/led_mode"));
  const auto color = static_cast<uint32_t>(get_command("led/led_color"));
  const auto color_alt = static_cast<uint32_t>(get_command("led/led_color_alt"));
  const double period_ms = get_command("led/led_period_ms");
  const auto led = ResolveLed(led_mode, color, color_alt, period_ms, time.seconds());
  if (!led) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Unknown LED mode %d; ignoring", led_mode);
    return;
  }

  // 0x36 has no ACK, so a dropped frame is only repaired by the 1 Hz refresh.
  const bool refresh_due = (get_clock()->now() - last_led_send_).seconds() >= 1.0;
  if (sent_led_ == led && !refresh_due) {
    return;
  }
  RCLCPP_DEBUG(get_logger(), "[LED] mode=%d rgb=%02X%02X%02X",
    led_mode, led->r, led->g, led->b);
  roboauto_.SetRGBLED(led->r, led->g, led->b, led->fw_mode);
  sent_led_ = led;
  last_led_send_ = get_clock()->now();
}

}  // namespace my_diffbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_diffbot_hardware_interface::My_diffbotSystemHardware,
                       hardware_interface::SystemInterface)

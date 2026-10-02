/*
 * Copyright (c) 2025 Ultra
 *
 * Author: Sayed ElSheikh
 */
#include "merlin_hardware_interface/merlin_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/introspection.hpp>
#include <rclcpp/logging.hpp>

namespace merlin_hardware_interface
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

// ACK wait for runtime retunes from write(): long enough for a healthy round
// trip, short enough not to stall the 100 Hz control loop (the activate-time
// sends keep the 3 s default).
constexpr uint32_t kRuntimeAckTimeoutMs = 20u;

// No WHEEL_FEEDBACK frame for this long → zero the measured wheel speed.
constexpr double kWheelFeedbackTimeoutSec = 0.5;

// gpio commands come from user messages: NaN/Inf/out-of-range must not reach a
// cast (UB). Non-finite maps to 0; finite values clamp to T's range.
template<typename T>
T ClampCast(double v)
{
  if (!std::isfinite(v)) {
    return T{};
  }
  return static_cast<T>(std::clamp(
    v, static_cast<double>(std::numeric_limits<T>::lowest()),
    static_cast<double>(std::numeric_limits<T>::max())));
}

constexpr std::array<const char *, 6> kGainOrder =
{"kp_l", "ki_l", "kd_l", "kp_r", "ki_r", "kd_r"};

// The ESP32 can still be booting (USB re-enumeration) when the controller
// manager starts — retry before failing.
bool HandshakeWithRetry(Kestrel & board, const rclcpp::Logger & logger)
{
  constexpr int kMaxAttempts = 10;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    if (board.Handshake() == AckStatus::OK) {
      return true;
    }
    RCLCPP_WARN(logger, "Kestrel: handshake attempt %d/%d failed", attempt, kMaxAttempts);
    if (attempt < kMaxAttempts) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  RCLCPP_ERROR(logger, "Kestrel: handshake failed after %d attempts!", kMaxAttempts);
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
hardware_interface::CallbackReturn MerlinSystemHardware::on_init(
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
  kp_l_ = param_int("kp_l", 1500);
  ki_l_ = param_int("ki_l", 3000);
  kd_l_ = param_int("kd_l", 0);
  kp_r_ = param_int("kp_r", 1500);
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
  kestrel_.SetClock(get_clock());
  kestrel_.Configure(serial_port, baud_rate, timeout_ms);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MerlinSystemHardware::on_configure(
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

  // A hardware component has no node of its own, so create a private one purely
  // as a publisher factory; publish() needs no executor/spin.
  diag_node_ = std::make_shared<rclcpp::Node>("merlin_hw_diagnostics");
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
hardware_interface::CallbackReturn MerlinSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Activating %s ...", get_name().c_str());

  // command and state should be equal when starting
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }

  if (!kestrel_.Connect()) {
    RCLCPP_ERROR(get_logger(), "Kestrel: serial port failed to open!");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  if (!HandshakeWithRetry(kestrel_, get_logger())) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  const rclcpp::Time now = get_clock()->now();
  last_active_time_ = now;
  last_imu_count_change_ = now;
  last_diag_pub_ = now;
  last_led_send_ = now;
  last_imu_count_ = 0;
  last_wheel_count_ = 0;
  last_wheel_count_change_ = now;
  wheel_feedback_stale_ = false;
  ticks_init_ = false;
  meas_rad_ = {0.0, 0.0};
  sent_led_.reset();

  // Non-fatal on a missing ACK: a flaky ACK shouldn't kill bringup, and the
  // kestrel_tuning_controller gpios can always re-send.
  RCLCPP_INFO(get_logger(),
    "Kestrel: sending PID gains L(%d, %d, %d) R(%d, %d, %d)",
    kp_l_, ki_l_, kd_l_, kp_r_, ki_r_, kd_r_);
  if (kestrel_.SetPIDGains(kp_l_, ki_l_, kd_l_, kp_r_, ki_r_, kd_r_) != AckStatus::OK) {
    RCLCPP_ERROR(get_logger(), "Kestrel: SetPIDGains not acknowledged!");
  }
  RCLCPP_INFO(get_logger(), "Kestrel: sending command timeout %u ms", cmd_timeout_ms_);
  if (kestrel_.SetCommandTimeout(cmd_timeout_ms_) != AckStatus::OK) {
    RCLCPP_ERROR(get_logger(), "Kestrel: SetCommandTimeout not acknowledged!");
  }
  sent_pid_gains_ = {
    static_cast<double>(kp_l_), static_cast<double>(ki_l_),
    static_cast<double>(kd_l_), static_cast<double>(kp_r_),
    static_cast<double>(ki_r_), static_cast<double>(kd_r_)};
  sent_cmd_timeout_ms_ = static_cast<double>(cmd_timeout_ms_);

  // Seed the tuning gpio commands with what was just sent, so neither the
  // configure-time zeroing nor the deactivate zeroing (or a stale runtime
  // retune) registers as a "change" on the first write().
  const std::array<std::pair<const char *, double>, 7> tuning_seeds = {{
    {"kestrel_pid/kp_l", sent_pid_gains_[0]},
    {"kestrel_pid/ki_l", sent_pid_gains_[1]},
    {"kestrel_pid/kd_l", sent_pid_gains_[2]},
    {"kestrel_pid/kp_r", sent_pid_gains_[3]},
    {"kestrel_pid/ki_r", sent_pid_gains_[4]},
    {"kestrel_pid/kd_r", sent_pid_gains_[5]},
    {"kestrel_watchdog/cmd_timeout_ms", sent_cmd_timeout_ms_},
  }};
  for (const auto & [name, value] : tuning_seeds) {
    if (gpio_command_interfaces_.count(name) != 0) {
      set_command(name, value);
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MerlinSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s ...", get_name().c_str());

  for (const auto &[name, descr] : gpio_command_interfaces_) {
    set_command(name, 0.0);
  }

  kestrel_.SetWheelVelocity(0.0, 0.0);
  kestrel_.Deactivate();
  kestrel_.Reset();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MerlinSystemHardware::on_cleanup(
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
  kestrel_.Shutdown();

  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type
MerlinSystemHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  if (!kestrel_.IsOpen()) {
    RCLCPP_ERROR(get_logger(), "Can't connect to Kestrel; serial port is not open!");
    return hardware_interface::return_type::ERROR;
  }

  // Drain the RX buffer and parse any new frames.
  kestrel_.Read();

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
  ReadBattery();

  if (publish_diag && !diag_array.status.empty()) {
    diag_array.header.stamp = diag_node_->get_clock()->now();
    diag_pub_->publish(diag_array);
  }

  return hardware_interface::return_type::OK;
}

void MerlinSystemHardware::ReadImu(
  bool publish_diag, diagnostic_msgs::msg::DiagnosticArray & diag_array)
{
  // Feed the bno055_imu sensor state interfaces (consumed by
  // imu_sensor_broadcaster → /imu/data → EKF). Quat order in the snapshot is w,x,y,z.
  const auto imu = kestrel_.GetIMUSnapshot();
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
  status.name = "merlin/imu: BNO055";
  status.hardware_id = "kestrel";
  if (!kestrel_.GetImuHealthy() || stale) {
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

// NaN until the first BATTERY_STATUS, so the broadcaster reports "not present", never 0 V.
void MerlinSystemHardware::ReadBattery()
{
  for (const auto &[name, descr] : sensor_state_interfaces_) {
    if (descr.get_prefix_name() == "battery_state") {
      set_state(name, kestrel_.GetBatteryVoltage());
    }
  }
}

void MerlinSystemHardware::ReadWheels(const rclcpp::Duration & /* period */)
{
  const auto feedback = kestrel_.GetWheelFeedbackSnapshot();

  // Introspection only: firmware speed (signed by the transport) and PWM duty.
  left_wheel_firmware_velocity_ = feedback.left_mrps;
  right_wheel_firmware_velocity_ = feedback.right_mrps;
  left_wheel_pwm_ = kestrel_.GetLeftDutyPermille();
  right_wheel_pwm_ = kestrel_.GetRightDutyPermille();

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
    meas_stamp_prev_ = feedback.stamp_sec;
    ticks_init_ = true;
    return;
  }

  if (feedback.count != last_wheel_count_) {
    last_wheel_count_ = feedback.count;
    last_wheel_count_change_ = get_clock()->now();
    if (wheel_feedback_stale_) {
      // Frames resumed: restart the speed window from this frame.
      wheel_feedback_stale_ = false;
      meas_left_prev_ = feedback.left_ticks;
      meas_right_prev_ = feedback.right_ticks;
      meas_stamp_prev_ = feedback.stamp_sec;
    }
  } else if ((get_clock()->now() - last_wheel_count_change_).seconds() > kWheelFeedbackTimeoutSec) {
    wheel_feedback_stale_ = true;
    meas_rad_ = {0.0, 0.0};
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Kestrel: wheel feedback stale (no frames > %.1f s); reporting zero wheel velocity",
      kWheelFeedbackTimeoutSec);
  }

  const int64_t left_delta = feedback.left_ticks - left_encoder_prev_;
  const int64_t right_delta = feedback.right_ticks - right_encoder_prev_;
  left_encoder_prev_ = feedback.left_ticks;
  right_encoder_prev_ = feedback.right_ticks;

  // Differentiate over >=40 ms of MCU frame time, then EMA-filter: ticks only
  // change when a WHEEL_FEEDBACK frame (~50 Hz) parses, so a per-cycle diff
  // aliases, and dividing by the jittery host period over-reads speed by ~5 %.
  constexpr double kMeasWindowSec = 0.04;
  constexpr double kMeasEmaAlpha = 0.4;
  const double window_sec = feedback.stamp_sec - meas_stamp_prev_;
  if (window_sec < 0.0 || window_sec > 1.0) {
    // MCU clock re-anchored (handshake) or a long gap: restart the window.
    meas_left_prev_ = feedback.left_ticks;
    meas_right_prev_ = feedback.right_ticks;
    meas_stamp_prev_ = feedback.stamp_sec;
  } else if (window_sec >= kMeasWindowSec) {
    const double scale = 2.0 * M_PI / (cpr_ * window_sec);
    meas_rad_[0] += kMeasEmaAlpha *
      (static_cast<double>(feedback.left_ticks - meas_left_prev_) * scale - meas_rad_[0]);
    meas_rad_[1] += kMeasEmaAlpha *
      (static_cast<double>(feedback.right_ticks - meas_right_prev_) * scale - meas_rad_[1]);
    meas_left_prev_ = feedback.left_ticks;
    meas_right_prev_ = feedback.right_ticks;
    meas_stamp_prev_ = feedback.stamp_sec;
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
MerlinSystemHardware::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & /* period */)
{
  if (!kestrel_.IsOpen()) {
    RCLCPP_ERROR(get_logger(), "Can't connect to Kestrel; serial port is not open!");
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
    if (kestrel_.SetWheelVelocity(left_cmd, right_cmd) != AckStatus::OK) {
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

void MerlinSystemHardware::WriteTuning()
{
  // GpioCommandController re-writes its last message every cycle, so only touch
  // the serial port when a value actually changes. Trackers update after the
  // send attempt regardless of ACK — a bad ACK must not retry at the full rate.
  std::array<double, 6> pid_cmd = sent_pid_gains_;
  double timeout_cmd = sent_cmd_timeout_ms_;
  for (const auto & [name, descr] : gpio_command_interfaces_) {
    const std::string prefix = descr.get_prefix_name();
    const std::string iface = descr.get_interface_name();
    if (prefix == "kestrel_pid") {
      for (size_t i = 0; i < kGainOrder.size(); ++i) {
        if (iface == kGainOrder[i] && std::isfinite(get_command(name))) {
          pid_cmd[i] = get_command(name);
        }
      }
    } else if (prefix == "kestrel_watchdog" && iface == "cmd_timeout_ms" &&
      std::isfinite(get_command(name)))
    {
      timeout_cmd = get_command(name);
    }
  }

  if (pid_cmd != sent_pid_gains_) {
    std::array<int32_t, 6> gains {};
    for (size_t i = 0; i < gains.size(); ++i) {
      gains[i] = ClampCast<int32_t>(pid_cmd[i]);
    }
    RCLCPP_INFO(get_logger(), "Kestrel: re-tuning PID gains L(%d, %d, %d) R(%d, %d, %d)",
      gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
    if (kestrel_.SetPIDGains(
        gains[0], gains[1], gains[2], gains[3], gains[4], gains[5],
        kRuntimeAckTimeoutMs) != AckStatus::OK)
    {
      RCLCPP_ERROR(get_logger(), "Kestrel: SetPIDGains not acknowledged!");
    }
    sent_pid_gains_ = pid_cmd;
  }

  if (timeout_cmd != sent_cmd_timeout_ms_) {
    const auto timeout_ms = ClampCast<uint16_t>(timeout_cmd);
    RCLCPP_INFO(get_logger(), "Kestrel: setting command timeout %u ms", timeout_ms);
    if (kestrel_.SetCommandTimeout(timeout_ms, kRuntimeAckTimeoutMs) != AckStatus::OK) {
      RCLCPP_ERROR(get_logger(), "Kestrel: SetCommandTimeout not acknowledged!");
    }
    sent_cmd_timeout_ms_ = timeout_cmd;
  }
}

void MerlinSystemHardware::WriteLed(const rclcpp::Time & time)
{
  if (gpio_command_interfaces_.count("led/led_mode") == 0u) {
    return;
  }
  const int led_mode = ClampCast<int>(get_command("led/led_mode"));
  const auto color = ClampCast<uint32_t>(get_command("led/led_color"));
  const auto color_alt = ClampCast<uint32_t>(get_command("led/led_color_alt"));
  const double period_ms = std::isfinite(get_command("led/led_period_ms")) ?
    get_command("led/led_period_ms") : 0.0;
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
  kestrel_.SetRGBLED(led->r, led->g, led->b, led->fw_mode);
  sent_led_ = led;
  last_led_send_ = get_clock()->now();
}

}  // namespace merlin_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(merlin_hardware_interface::MerlinSystemHardware,
                       hardware_interface::SystemInterface)

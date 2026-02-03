/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: Sayed ElSheikh
 */
#include "my_diffbot_hardware_interface/my_diffbot_system.hpp"
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/logging.hpp>

namespace my_diffbot_hardware_interface
{

hardware_interface::CallbackReturn My_diffbotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Configuring %s ...", get_name().c_str());

  // Ensure disconnection before configuring
  if (serial_comms_.is_open()) {
    serial_comms_.disconnect();
  }

  // reset values always when configuring hardware
  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }
  for (const auto &[name, descr] : sensor_state_interfaces_) {
    set_state(name, 0.0);
  }

  // Connect to the robot
  if (serial_comms_.connect()) {
    RCLCPP_INFO(get_logger(), "Successfully connected to the robot!");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to connect to the robot!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn My_diffbotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Activating %s ...", get_name().c_str());

  // command and state should be equal when starting
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }

  if (!serial_comms_.is_open()) {
    RCLCPP_ERROR(get_logger(),
                 "Cannot activate because not connected to the robot!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn My_diffbotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s ...", get_name().c_str());
  serial_comms_.reset_command();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn My_diffbotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s ...", get_name().c_str());
  if (serial_comms_.is_open()) {
    serial_comms_.disconnect();
  }
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// SystemInterface methods
hardware_interface::CallbackReturn My_diffbotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string serial_port = info_.hardware_parameters["serial_port"];
  int32_t baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  int32_t encoder_resolution =
    std::stoi(info_.hardware_parameters["encoder_counts_per_revolution"]);
  int32_t timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // DiffBotSystem has exactly two states and one command interface on each
  // joint
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


  for (const hardware_interface::ComponentInfo & sensor: info_.sensors) {
    if (sensor.state_interfaces.size() != 1) {
      RCLCPP_FATAL(get_logger(),
          "Sensor '%s' has %zu state interfaces; 1 expected.",
          sensor.name.c_str(), sensor.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (sensor.state_interfaces[0].name != "range") {
      RCLCPP_FATAL(get_logger(),
          "Sensor '%s' has '%s' state interface; 'range' expected.",
          sensor.name.c_str(),
          sensor.state_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Configure serial communications
  serial_comms_.configure(serial_port, baud_rate, timeout_ms, encoder_resolution);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
My_diffbotSystemHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & period)
{
  if (!serial_comms_.is_open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot read from serial; not connected to the robot!");
    return hardware_interface::return_type::ERROR;
  }
  // Read Ultrasonic data & convert it to meters
  int ultrasonic_data = 0;
  serial_comms_.read_ultrasonic_value(ultrasonic_data);
  ultrasonic_range_= ultrasonic_data / 100.0;

  for (const auto &[name, descr] : sensor_state_interfaces_) {
    if (descr.get_interface_name() == "range") {
      set_state(name, ultrasonic_range_);
      RCLCPP_DEBUG(get_logger(), "Sensor '%s' range: %.2f m", descr.get_prefix_name().c_str(), ultrasonic_range_);
    }
  }

  // Read encoder values from hardware
  int64_t left_encoder_counts = 0;
  int64_t right_encoder_counts = 0;
  serial_comms_.read_encoder_values(left_encoder_counts, right_encoder_counts);

  double delta_seconds = period.seconds();

  // Calculate encoder deltas (handles overflow/underflow)
  int64_t left_delta = left_encoder_counts - left_encoder_prev_;
  int64_t right_delta = right_encoder_counts - right_encoder_prev_;

  // Update previous values for next iteration
  left_encoder_prev_ = left_encoder_counts;
  right_encoder_prev_ = right_encoder_counts;

  // Update each joint's position and velocity
  for (const auto &[name, descr] : joint_state_interfaces_) {
    const std::string prefix = descr.get_prefix_name();

    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      // Store previous position for velocity calculation
      double pos_prev = get_state(name);

      // Calculate position change from encoder delta
      int64_t encoder_delta = 0;
      if (prefix.find("left") != std::string::npos) {
        encoder_delta = left_delta;
      } else if (prefix.find("right") != std::string::npos) {
        encoder_delta = right_delta;
      }

      double position_delta = serial_comms_.encoder_counts_to_radians(encoder_delta);

      // Update position (cumulative)
      double new_position = pos_prev + position_delta;
      set_state(name, new_position);

      // Calculate and update velocity
      std::string velocity_interface_name = prefix + "/" + hardware_interface::HW_IF_VELOCITY;
      double velocity = position_delta / delta_seconds;
      set_state(velocity_interface_name, velocity);

      RCLCPP_DEBUG(
        get_logger(), "%s: pos=%.3f rad, vel=%.3f rad/s",
        prefix.c_str(), new_position, velocity);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
My_diffbotSystemHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (!serial_comms_.is_open()) {
    RCLCPP_ERROR(get_logger(),
                 "Cannot write to serial; not connected to the robot!");
    return hardware_interface::return_type::ERROR;
  }

  double left_val = 0;
  double right_val = 0;
  for (const auto &[name, descr] : joint_command_interfaces_) {
    const std::string prefix = descr.get_prefix_name();
    double cmd = get_command(name);
    if (prefix.find("left") != std::string::npos) {
      left_val = cmd;
    } else if (prefix.find("right") != std::string::npos) {
      right_val = cmd;
    } else {
      RCLCPP_WARN(get_logger(),
                  "Unknown joint '%s' when mapping command to motor",
                  prefix.c_str());
    }
  }
  serial_comms_.set_motor_speeds(left_val, right_val);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Sending commands: left_wheel_velocity = %.2f, "
    "right_wheel_velocity = %.2f",
    left_val, right_val);
  return hardware_interface::return_type::OK;
}

}  // namespace my_diffbot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_diffbot_hardware_interface::My_diffbotSystemHardware,
                       hardware_interface::SystemInterface)

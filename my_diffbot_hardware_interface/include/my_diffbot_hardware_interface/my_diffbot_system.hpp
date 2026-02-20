/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: Sayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "my_diffbot_hardware_interface/serial_communications.hpp"

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
  SerialComms serial_comms_;
  int64_t left_encoder_prev_ = 0;
  int64_t right_encoder_prev_ = 0;
  double ultrasonic_range_ = 0.0;
  bool use_ultrasonic_ = false;
};

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__MY_DIFFBOT_SYSTEM_HPP_

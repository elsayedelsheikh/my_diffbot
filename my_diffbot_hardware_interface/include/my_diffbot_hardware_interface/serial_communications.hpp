/*
 * Copyright (c) 2025 RoboLabs
 *
 * Author: Sayed ElSheikh
 */
#ifndef MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_COMMUNICATIONS_HPP_
#define MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_COMMUNICATIONS_HPP_

#include <libserial/SerialPort.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

namespace my_diffbot_hardware_interface
{

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate) {
    case 1200:
      return LibSerial::BaudRate::BAUD_1200;
    case 1800:
      return LibSerial::BaudRate::BAUD_1800;
    case 2400:
      return LibSerial::BaudRate::BAUD_2400;
    case 4800:
      return LibSerial::BaudRate::BAUD_4800;
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate
                << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class SerialComms {
public:
  SerialComms() = default;
  ~SerialComms() {disconnect();}

  /// @brief Configure the serial communication parameters
  /// @param serial_port The serial port to use (e.g., "/dev/ttyUSB0")
  /// @param baud_rate  The baud rate for the serial communication (e.g., 57600)
  /// @param read_timeout_ms The read timeout in milliseconds
  /// @param encoder_cpr The encoder counts per revolution
  /// @param mcu_loop_rate The MCU loop rate in Hz (default is 30Hz)
  void configure(
    const std::string & serial_port, int baud_rate,
    int read_timeout_ms, int encoder_cpr, int mcu_loop_rate = 30)
  {
    port_ = serial_port;
    baud_rate_ = baud_rate;
    timeout_ms_ = read_timeout_ms;
    mcu_loop_rate_ = mcu_loop_rate;
    rads_per_count_ = calculate_rads_per_count(encoder_cpr);

    RCLCPP_INFO(
      logger_,
      "Configured serial: port=%s, baud=%d, encoder_cpr=%d",
      port_.c_str(), baud_rate_, encoder_cpr);
    RCLCPP_DEBUG(logger_, "Calculated rads_per_count = %.6f", rads_per_count_);
  }

  /// @brief Set the logger for debug output
  void set_logger(rclcpp::Logger logger) {logger_ = logger;}

  /// @brief Establish the serial connection
  /// @return true if the connection was successful, false otherwise
  bool connect()
  {
    try {
      if (is_open()) {
        return true;
      }
      serial_conn_.Open(port_);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate_));
      RCLCPP_INFO(logger_, "Connected to serial port: %s", port_.c_str());
      return is_open();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to connect to %s: %s", port_.c_str(), e.what());
      return false;
    }
  }

  void disconnect() {reset_command(); serial_conn_.Close();}

  /// @brief Check if the serial connection is open
  /// @return true if the connection is open, false otherwise
  bool is_open() const {return serial_conn_.IsOpen();}

  /// @brief Send command to stop the motors and reset encoder readings
  void reset_command()
  {
    set_motor_values(0, 0);
    send_command("\r");
  }

  /// @brief Set motor speeds in radians per second
  ///
  /// Converts angular velocity (rad/s) to counts per loop and sends the command
  /// to the motor controller. The conversion uses the encoder resolution and loop rate.
  ///
  /// @param left_motor_rad_per_sec Desired speed for left motor in radians per second
  /// @param right_motor_rad_per_sec Desired speed for right motor in radians per second
  void set_motor_speeds(double left_motor_rad_per_sec, double right_motor_rad_per_sec)
  {
    // 1. Calculate the raw floating point counts needed for this loop
    double l_raw = (left_motor_rad_per_sec / rads_per_count_ / mcu_loop_rate_) + left_remainder_;
    double r_raw = (right_motor_rad_per_sec / rads_per_count_ / mcu_loop_rate_) + right_remainder_;

    // 2. Convert to int the MCU expects
    int l_cmd = static_cast<int>(std::round(l_raw));
    int r_cmd = static_cast<int>(std::round(r_raw));

    // 3. Save the difference (the part we couldn't send) for the next loop
    left_remainder_ = l_raw - l_cmd;
    right_remainder_ = r_raw - r_cmd;
    set_motor_values(l_cmd, r_cmd);
  }

  /// @brief Read encoder values for left and right motors
  ///
  /// Sends the "e\r" command to the motor controller and parses the response
  /// in the format "left right" (space-separated integer values).
  ///
  /// @param leftMotorEncoder Reference to variable that will be updated with left encoder count
  /// @param rightMotorEncoder Reference to variable that will be updated with right encoder count
  void read_encoder_values(int64_t & leftMotorEncoder, int64_t & rightMotorEncoder)
  {
    // Send command to read encoders & Read response
    std::string response = "";
    send_command("e\r", &response);

    // Parse response
    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    leftMotorEncoder = std::atol(token_1.c_str());
    rightMotorEncoder = std::atol(token_2.c_str());
  }

  /// @brief Convert encoder counts to radians
  /// @param encoder_counts The raw encoder count value
  /// @return Angular position in radians
  double encoder_counts_to_radians(int64_t encoder_counts) const
  {
    return encoder_counts * rads_per_count_;
  }

private:
  /// @brief Calculate radians per encoder count based on wheel and encoder specs
  /// @param encoder_cpr Encoder counts per revolution (CPR or PPR)
  /// @return Radians traveled per single encoder count
  inline double calculate_rads_per_count(int encoder_cpr)
  {
    // One full revolution = 2π radians
    return (2.0 * M_PI) / (encoder_cpr);
  }

  /// @brief Set motor values for left and right motors in counts per loop
  ///
  /// Sends the "m left right\r" command to the motor controller to set
  /// the closed-loop speed of each motor in counts per loop.
  ///
  /// Note: counts_per_loop = encoder_counts_per_second / loop_rate
  /// (Default loop rate is 30Hz, so divide your desired counts/sec by 30)
  ///
  /// @param left_motor_value Speed for the left motor in counts per loop
  /// @param right_motor_value Speed for the right motor in counts per loop
  void set_motor_values(int left_motor_value, int right_motor_value)
  {
    std::stringstream ss;
    ss << "m " << left_motor_value << " " << right_motor_value << "\r";
    // RCLCPP_INFO(logger_, "SEND COMMAND: %s", ss.str().c_str());
    send_command(ss.str());
  }

  /// @brief Flush input and output buffers of the serial connection
  void flush_buffers() {serial_conn_.FlushIOBuffers();}

  /// @brief Send a command and optionally receive a response
  /// @param msg_to_send The command message to send
  /// @param response Pointer to string to store the response (if nullptr, no response is read)
  void send_command(const std::string & msg_to_send, std::string * response = nullptr)
  {
    try {
      this->flush_buffers();
      RCLCPP_DEBUG(logger_, "Sending hardware command: %s", msg_to_send.c_str());
      serial_conn_.Write(msg_to_send);

      // Read response if requested
      if (response != nullptr) {
        serial_conn_.ReadLine(*response, '\n', timeout_ms_);
        RCLCPP_DEBUG(logger_, "Received hardware response: %s", response->c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to send hardware command: %s", e.what());
    }
  }

  LibSerial::SerialPort serial_conn_;
  std::string port_;
  int32_t baud_rate_;
  int32_t timeout_ms_;
  double rads_per_count_;
  int mcu_loop_rate_;
  rclcpp::Logger logger_ {rclcpp::get_logger("SerialCommunications")};
  double left_remainder_ = 0.0;
  double right_remainder_ = 0.0;
};

}  // namespace my_diffbot_hardware_interface

#endif  // MY_DIFFBOT_HARDWARE_INTERFACE__SERIAL_COMMUNICATIONS_HPP_

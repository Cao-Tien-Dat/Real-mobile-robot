#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diffdrive_arduino/wheel.h"
#include "diffdrive_arduino/arduino_comms.h"
#include "diffdrive_arduino/visibility_control.h"

using hardware_interface::return_type;

class DiffDriveArduino
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  struct Config
  {
    std::string left_wheel_name = "khop_banh_trai";
    std::string right_wheel_name = "khop_banh_phai";
    float loop_rate = 100.0;  // 
    std::string device = "/dev/ttyUSB0";  // default serial port
    int baud_rate = 115200;  // default baud rate
    int timeout_ms = 1000;
    int enc_counts_per_rev = 2970;
    int pid_p = 20;
    int pid_d = 12;
    int pid_i = 1;
  };

  DiffDriveArduino();

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type start() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type stop() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type read() override;

  DIFFDRIVE_ARDUINO_PUBLIC
  hardware_interface::return_type write() override;

private:
  Config cfg_;
  ArduinoComms comms_;

  Wheel wheel_l_;
  Wheel wheel_r_;

  rclcpp::Logger logger_ = rclcpp::get_logger("DiffDriveArduino");

  std::chrono::time_point<std::chrono::system_clock> time_last_;  // để tính period thủ công
};

#endif  // DIFFDRIVE_ARDUINO_REAL_ROBOT_H

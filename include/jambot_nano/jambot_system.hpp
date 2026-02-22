// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JAMBOT_NANO__DIFFBOT_SYSTEM_HPP_
#define JAMBOT_NANO__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "jambot_nano/visibility_control.h"

#include "jambot_nano/arduino_comms.hpp"
#include "jambot_nano/wheel.hpp"

namespace jambot_nano
{
class JamBotNanoHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JamBotNanoHardware)

  JAMBOT_NANO_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  JAMBOT_NANO_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  JAMBOT_NANO_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  JAMBOT_NANO_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  JAMBOT_NANO_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  JAMBOT_NANO_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  JAMBOT_NANO_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  JAMBOT_NANO_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  JAMBOT_NANO_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  double battery_voltage_ = 0.0;  // Battery voltage state
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr buzzer_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr led_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::atomic<int> buzzer_mode_{0};
  std::atomic<bool> buzzer_cmd_pending_{false};
  std::atomic<int> led_r_{0};
  std::atomic<int> led_g_{0};
  std::atomic<int> led_b_{0};
  std::atomic<bool> led_cmd_pending_{false};
};

}  // namespace jambot_nano

#endif  // JAMBOT_NANO__DIFFBOT_SYSTEM_HPP_

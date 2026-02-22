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

#include "jambot_nano/jambot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace jambot_nano
{
hardware_interface::CallbackReturn JamBotNanoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "PID values not supplied, using defaults.");
  }

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JamBotNanoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name_, hardware_interface::HW_IF_POSITION, &wheel_l_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name_, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name_, hardware_interface::HW_IF_POSITION, &wheel_r_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name_, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel_));

  // Add battery voltage state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "battery", "voltage", &battery_voltage_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JamBotNanoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name_, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name_, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn JamBotNanoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  if (!comms_.connected()) {
    RCLCPP_ERROR(rclcpp::get_logger("JamBotNanoHardware"), 
      "Failed to connect to any serial port.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  io_node_ = rclcpp::Node::make_shared("jambot_hardware_io");
  imu_pub_ = io_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
  buzzer_sub_ = io_node_->create_subscription<std_msgs::msg::Int32>(
    "/jambot/buzzer_mode",
    10,
    [this](const std_msgs::msg::Int32::SharedPtr msg)
    {
      buzzer_mode_.store(msg->data);
      buzzer_cmd_pending_.store(true);
      RCLCPP_INFO(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Queued buzzer mode command: %d",
        msg->data);
    });
  led_sub_ = io_node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/jambot/led_rgb",
    10,
    [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
      if (msg->data.size() < 3) {
        RCLCPP_WARN(
          rclcpp::get_logger("JamBotNanoHardware"),
          "Ignoring LED command with %zu values; expected 3",
          msg->data.size());
        return;
      }
      const int red = std::clamp(msg->data[0], 0, 1);
      const int green = std::clamp(msg->data[1], 0, 1);
      const int blue = std::clamp(msg->data[2], 0, 1);
      led_r_.store(red);
      led_g_.store(green);
      led_b_.store(blue);
      led_cmd_pending_.store(true);
      RCLCPP_INFO(
        rclcpp::get_logger("JamBotNanoHardware"),
        "Queued LED command: r=%d g=%d b=%d",
        red, green, blue);
    });
  
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JamBotNanoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  imu_pub_.reset();
  led_sub_.reset();
  buzzer_sub_.reset();
  io_node_.reset();
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JamBotNanoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JamBotNanoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Deactivating ...please wait...");
  if (comms_.connected())
  {
    comms_.set_motor_values(0.0, 0.0);
  }
  RCLCPP_INFO(rclcpp::get_logger("JamBotNanoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type JamBotNanoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (io_node_)
  {
    rclcpp::spin_some(io_node_);
  }

  // Firmware returns encoder counts in opposite wheel order.
  // Swap assignment so odometry yaw sign matches real turns.
  comms_.read_encoder_values(wheel_r_.enc_, wheel_l_.enc_);

  /* Correct encoder polarity so forward physical motion is +X in odometry. */
  wheel_l_.enc_ = -wheel_l_.enc_;
  wheel_r_.enc_ = -wheel_r_.enc_;

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos_;
  wheel_l_.pos_ = wheel_l_.calc_enc_angle();
  wheel_l_.vel_ = (wheel_l_.pos_ - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos_;
  wheel_r_.pos_ = wheel_r_.calc_enc_angle();
  wheel_r_.vel_ = (wheel_r_.pos_ - pos_prev) / delta_seconds;

  // Read battery voltage
  battery_voltage_ = comms_.read_battery_voltage();

  float ax_raw = 0.0f;
  float ay_raw = 0.0f;
  float az_raw = 0.0f;
  float gx_raw = 0.0f;
  float gy_raw = 0.0f;
  float gz_raw = 0.0f;
  comms_.read_imu_data(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
  if (imu_pub_ && std::isfinite(ax_raw) && std::isfinite(ay_raw) && std::isfinite(az_raw) &&
      std::isfinite(gx_raw) && std::isfinite(gy_raw) && std::isfinite(gz_raw))
  {
    // Convert MPU6050 raw counts using default sensitivities (2g, 250dps).
    constexpr double kAccelScale = 9.80665 / 16384.0;
    constexpr double kGyroScale = (M_PI / 180.0) / 131.0;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = io_node_->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = static_cast<double>(ax_raw) * kAccelScale;
    imu_msg.linear_acceleration.y = static_cast<double>(ay_raw) * kAccelScale;
    imu_msg.linear_acceleration.z = static_cast<double>(az_raw) * kAccelScale;
    imu_msg.angular_velocity.x = static_cast<double>(gx_raw) * kGyroScale;
    imu_msg.angular_velocity.y = static_cast<double>(gy_raw) * kGyroScale;
    imu_msg.angular_velocity.z = static_cast<double>(gz_raw) * kGyroScale;

    imu_msg.orientation_covariance[0] = -1.0;  // Orientation unavailable.
    imu_pub_->publish(imu_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JamBotNanoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (io_node_)
  {
    rclcpp::spin_some(io_node_);
  }

  if (buzzer_cmd_pending_.exchange(false))
  {
    const int mode = buzzer_mode_.load();
    comms_.play_sound(mode);
    RCLCPP_INFO(
      rclcpp::get_logger("JamBotNanoHardware"),
      "Sent buzzer mode command: %d",
      mode);
  }
  if (led_cmd_pending_.exchange(false))
  {
    const int red = led_r_.load();
    const int green = led_g_.load();
    const int blue = led_b_.load();
    comms_.set_led_state(red, green, blue);
    RCLCPP_INFO(
      rclcpp::get_logger("JamBotNanoHardware"),
      "Sent LED command: r=%d g=%d b=%d",
      red, green, blue);
  }

  comms_.set_motor_values(wheel_l_.cmd_, wheel_r_.cmd_);
  return hardware_interface::return_type::OK;
}

}  // namespace jambot_nano

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  jambot_nano::JamBotNanoHardware, hardware_interface::SystemInterface)

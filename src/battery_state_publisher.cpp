#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class BatteryStatePublisher : public rclcpp::Node
{
public:
  BatteryStatePublisher()
  : Node("battery_state_publisher")
  {
    min_voltage_ = this->declare_parameter<double>("min_voltage", 9.0);
    max_voltage_ = this->declare_parameter<double>("max_voltage", 12.6);
    battery_component_name_ = this->declare_parameter<std::string>("battery_component_name", "battery");
    battery_interface_name_ = this->declare_parameter<std::string>("battery_interface_name", "voltage");

    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
    dynamic_joint_state_sub_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states",
      10,
      std::bind(&BatteryStatePublisher::dynamic_joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Listening on /dynamic_joint_states for %s/%s and publishing /battery_state",
      battery_component_name_.c_str(),
      battery_interface_name_.c_str());
  }

private:
  void dynamic_joint_state_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    const auto joint_count = msg->joint_names.size();
    const auto iface_count = msg->interface_values.size();
    const auto count = std::min(joint_count, iface_count);

    for (size_t i = 0; i < count; ++i) {
      if (msg->joint_names[i] != battery_component_name_) {
        continue;
      }

      const auto & interfaces = msg->interface_values[i].interface_names;
      const auto & values = msg->interface_values[i].values;
      const auto value_count = std::min(interfaces.size(), values.size());

      for (size_t j = 0; j < value_count; ++j) {
        if (interfaces[j] != battery_interface_name_) {
          continue;
        }

        const double voltage = values[j];
        if (std::isnan(voltage) || std::isinf(voltage)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Battery voltage is invalid (NaN/inf), skipping publish");
          return;
        }

        sensor_msgs::msg::BatteryState out;
        out.header = msg->header;
        out.voltage = static_cast<float>(voltage);
        out.current = std::numeric_limits<float>::quiet_NaN();
        out.charge = std::numeric_limits<float>::quiet_NaN();
        out.capacity = std::numeric_limits<float>::quiet_NaN();
        out.design_capacity = std::numeric_limits<float>::quiet_NaN();
        out.temperature = std::numeric_limits<float>::quiet_NaN();
        out.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        out.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        out.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        out.present = true;

        if (max_voltage_ > min_voltage_) {
          const double normalized = (voltage - min_voltage_) / (max_voltage_ - min_voltage_);
          out.percentage = static_cast<float>(std::clamp(normalized, 0.0, 1.0));
        } else {
          out.percentage = std::numeric_limits<float>::quiet_NaN();
        }

        battery_pub_->publish(out);
        return;
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_state_sub_;
  double min_voltage_{9.0};
  double max_voltage_{12.6};
  std::string battery_component_name_{"battery"};
  std::string battery_interface_name_{"voltage"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatePublisher>());
  rclcpp::shutdown();
  return 0;
}

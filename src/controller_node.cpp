#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <vector>
#include <string>

class TriggerControlNode : public rclcpp::Node
{
public:
    TriggerControlNode() : Node("trigger_control_node"),
                           current_linear_speed_(0.10),
                           current_angular_speed_(1.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/jambot_base_controller/cmd_vel", 10);
        buzzer_pub_ = this->create_publisher<std_msgs::msg::Int32>("/jambot/buzzer_mode", 10);
        led_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/jambot/led_rgb", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TriggerControlNode::joy_callback, this, std::placeholders::_1));

        twist_stamped_ = geometry_msgs::msg::TwistStamped();
        RCLCPP_INFO(this->get_logger(), "Buzzer controls: Y=toggle, L1/L2=mode up/down");
        RCLCPP_INFO(this->get_logger(), "LED controls: B=shuffle colors");
    }

private:
    void publish_buzzer_mode(int mode)
    {
        std_msgs::msg::Int32 msg;
        msg.data = mode;
        buzzer_pub_->publish(msg);
    }

    void publish_led_color(int red, int green, int blue)
    {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {red, green, blue};
        led_pub_->publish(msg);
    }

    static bool rising_edge(const sensor_msgs::msg::Joy::SharedPtr &msg,
                            const std::vector<int> &prev_buttons,
                            size_t idx)
    {
        if (idx >= msg->buttons.size() || idx >= prev_buttons.size()) {
            return false;
        }
        return msg->buttons[idx] == 1 && prev_buttons[idx] == 0;
    }

    static float get_axis_or_zero(const sensor_msgs::msg::Joy::SharedPtr &msg, size_t idx)
    {
        if (idx >= msg->axes.size()) {
            return 0.0f;
        }
        return msg->axes[idx];
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.assign(msg->buttons.size(), 0);
        }

        constexpr size_t BUTTON_Y = 3;
        constexpr size_t BUTTON_L1 = 4;
        constexpr size_t BUTTON_L2 = 5;
        constexpr size_t BUTTON_B = 1;
        constexpr int kMinMode = 1;
        constexpr int kMaxMode = 8;

        if (rising_edge(msg, prev_buttons_, BUTTON_Y)) {
            buzzer_enabled_ = !buzzer_enabled_;
            if (buzzer_enabled_) {
                // Current firmware treats n=1 as ON and any other value as OFF.
                publish_buzzer_mode(1);
                RCLCPP_INFO(this->get_logger(), "Y pressed: buzzer ON, mode=%d", buzzer_mode_);
            } else {
                publish_buzzer_mode(0);
                RCLCPP_INFO(this->get_logger(), "Y pressed: buzzer OFF");
            }
        }

        if (rising_edge(msg, prev_buttons_, BUTTON_L1)) {
            buzzer_mode_++;
            if (buzzer_mode_ > kMaxMode) {
                buzzer_mode_ = kMinMode;
            }
            RCLCPP_INFO(this->get_logger(), "L1 pressed: buzzer mode=%d", buzzer_mode_);
            if (buzzer_enabled_) {
                publish_buzzer_mode(1);
            }
        }

        if (rising_edge(msg, prev_buttons_, BUTTON_L2)) {
            buzzer_mode_--;
            if (buzzer_mode_ < kMinMode) {
                buzzer_mode_ = kMaxMode;
            }
            RCLCPP_INFO(this->get_logger(), "L2 pressed: buzzer mode=%d", buzzer_mode_);
            if (buzzer_enabled_) {
                publish_buzzer_mode(1);
            }
        }

        if (rising_edge(msg, prev_buttons_, BUTTON_B)) {
            led_color_index_ = (led_color_index_ + 1) % static_cast<int>(led_colors_.size());
            const auto &color = led_colors_[led_color_index_];
            publish_led_color(color[0], color[1], color[2]);
            RCLCPP_INFO(
                this->get_logger(),
                "B pressed: LED color -> %s (%d,%d,%d)",
                led_color_names_[led_color_index_].c_str(),
                color[0], color[1], color[2]);
        }

        float dpad_horizontal = get_axis_or_zero(msg, 6);
        float dpad_vertical = get_axis_or_zero(msg, 7);

        if (dpad_vertical > 0.0)
        {
            current_linear_speed_ += 0.05;
            current_linear_speed_ = std::min(current_linear_speed_, 1.0f);
            RCLCPP_INFO(this->get_logger(), "Increased linear speed scale: %.2f", current_linear_speed_);
        }
        else if (dpad_vertical < 0.0)
        {
            current_linear_speed_ -= 0.05;
            current_linear_speed_ = std::max(current_linear_speed_, 0.1f);
            RCLCPP_INFO(this->get_logger(), "Decreased linear speed scale: %.2f", current_linear_speed_);
        }

        if (dpad_horizontal > 0.0)
        {
            current_angular_speed_ -= 0.05;
            current_angular_speed_ = std::min(current_angular_speed_, 1.0f);
            RCLCPP_INFO(this->get_logger(), "Increased angular speed scale: %.2f", current_angular_speed_);
        }
        else if (dpad_horizontal < 0.0)
        {
            current_angular_speed_ += 0.05;
            current_angular_speed_ = std::max(current_angular_speed_, 0.1f);
            RCLCPP_INFO(this->get_logger(), "Decreased angular speed scale: %.2f", current_angular_speed_);
        }

        float right_trigger = get_axis_or_zero(msg, 5);
        twist_stamped_.twist.linear.x = current_linear_speed_ * right_trigger;

        float left_trigger = get_axis_or_zero(msg, 4);
        twist_stamped_.twist.linear.x += current_linear_speed_ * (-left_trigger);

        twist_stamped_.twist.angular.z = current_angular_speed_ * get_axis_or_zero(msg, 0);

        twist_stamped_.header.stamp = this->get_clock()->now();
        twist_stamped_.header.frame_id = "base_link";

        publisher_->publish(twist_stamped_);
        prev_buttons_ = msg->buttons;
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buzzer_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr led_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    geometry_msgs::msg::TwistStamped twist_stamped_;

    float current_linear_speed_;
    float current_angular_speed_;
    std::vector<int> prev_buttons_;
    bool buzzer_enabled_{false};
    int buzzer_mode_{1};
    int led_color_index_{0};
    const std::vector<std::array<int, 3>> led_colors_{
        {1, 0, 0},  // red
        {0, 1, 0},  // green
        {0, 0, 1},  // blue
        {1, 1, 0},  // yellow
        {1, 0, 1},  // magenta
        {0, 1, 1},  // cyan
        {1, 1, 1},  // white
        {0, 0, 0},  // off
    };
    const std::vector<std::string> led_color_names_{
        "red", "green", "blue", "yellow", "magenta", "cyan", "white", "off"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TriggerControlNode>());
    rclcpp::shutdown();
    return 0;
}

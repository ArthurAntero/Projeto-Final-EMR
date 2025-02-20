#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <iostream>

class IsBatteryAbove15 : public BT::ConditionNode, public rclcpp::Node
{
public:
    IsBatteryAbove15(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), rclcpp::Node("is_battery_above_15")
    {
        // Assina o tópico correto `/battery_state`
        battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 10, std::bind(&IsBatteryAbove15::batteryCallback, this, std::placeholders::_1));

        battery_level_ = 100.0; // Inicializa a bateria como cheia
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<double>("battery_level")};
    }

    BT::NodeStatus tick() override
    {
        setOutput("battery_level", battery_level_);

        if (battery_level_ > 15.0)
        {
            RCLCPP_INFO(this->get_logger(), "Battery level is above 15%% (%.2f%%)", battery_level_);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_WARN(this->get_logger(), "Battery level is below or equal to 15%% (%.2f%%)", battery_level_);
        return BT::NodeStatus::FAILURE;
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        battery_level_ = msg->percentage * 100.0; // Converte de fração para porcentagem
    }

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
    double battery_level_;
};

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class IsPickUpAvailable : public BT::ConditionNode, public rclcpp::Node
{
public:
    IsPickUpAvailable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), rclcpp::Node("is_pick_up_available_node") {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, bool>>("available"),
            BT::InputPort<std::unordered_map<int, int>>("meals")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, bool> available;
        std::unordered_map<int, int> meals;

        if (!getInput("available", available))
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] IsPickUpAvailable: Failed to get pickup availability map from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("meals", meals))
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] IsPickUpAvailable: Failed to get meals map from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[WARNING] IsPickUpAvailable: No meals available");
            return BT::NodeStatus::FAILURE;
        }

        int room_id = meals.begin()->second;

        auto it = available.find(room_id);
        if (it == available.end())
        {
            RCLCPP_WARN(this->get_logger(), "[WARNING] IsPickUpAvailable: Room %d not found in availability map", room_id);
            return BT::NodeStatus::FAILURE;
        }

        if (it->second)
        {
            RCLCPP_INFO(this->get_logger(), "[INFO] IsPickUpAvailable: Pickup available for room %d", room_id);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_WARN(this->get_logger(), "[WARNING] IsPickUpAvailable: Pickup not available for room %d", room_id);
        return BT::NodeStatus::FAILURE;
    }
};
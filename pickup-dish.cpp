#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

class PickUpDish : public BT::SyncActionNode, public rclcpp::Node
{
public:
    PickUpDish(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("pick_up_dish_node") {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<int>>("dishes")
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<int> dishes;
        if (!getInput("dishes", dishes))
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] PickUpDish: Failed to get dishes list");
            return BT::NodeStatus::FAILURE;
        }

        if (dishes.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[WARNING] PickUpDish: No dishes available to pick up");
            return BT::NodeStatus::FAILURE;
        }

        int room_id = dishes.front();
        dishes.erase(dishes.begin());

        if (auto bb = config().blackboard)
        {
            bb->set("dishes", dishes);
        }

        RCLCPP_INFO(this->get_logger(), "[INFO] PickUpDish: Collected dirty dishes from room %d", room_id);
        return BT::NodeStatus::SUCCESS;
    }
};
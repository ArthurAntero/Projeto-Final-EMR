#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <thread>

class WaitForPickUp : public BT::SyncActionNode, public rclcpp::Node
{
public:
    WaitForPickUp(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("wait_for_pickup_node") {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("pickup_time_sec")};
    }

    BT::NodeStatus tick() override
    {
        int pickup_time_sec;
        if (!getInput("pickup_time_sec", pickup_time_sec))
        {
            RCLCPP_ERROR(this->get_logger(), "WaitForPickup: Failed to get pickup_time_sec");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "WaitForPickup: Waiting for pickup for %d seconds...", pickup_time_sec);
        std::this_thread::sleep_for(std::chrono::seconds(pickup_time_sec));

        RCLCPP_INFO(this->get_logger(), "WaitForPickup: Pickup wait completed.");
        return BT::NodeStatus::SUCCESS;
    }
};
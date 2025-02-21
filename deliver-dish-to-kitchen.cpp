#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

class DeliverDishesToKitchen : public BT::SyncActionNode, public rclcpp::Node
{
public:
    DeliverDishesToKitchen(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("deliver_dishes_to_kitchen_node") {}

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
            RCLCPP_ERROR(this->get_logger(), "[ERROR] DeliverDishesToKitchen: Failed to get dishes list");
            return BT::NodeStatus::FAILURE;
        }

        if (dishes.empty())
        {
            RCLCPP_WARN(this->get_logger(), "[WARNING] DeliverDishesToKitchen: No dishes to deliver");
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(this->get_logger(), "Delivering dishes to kitchen...");

        while (!dishes.empty())
        {
            int room_id = dishes.front();
            dishes.erase(dishes.begin());
            RCLCPP_INFO(this->get_logger(), "Delivering dish from room %d... Done.", room_id);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        RCLCPP_INFO(this->get_logger(), "[INFO] DeliverDishesToKitchen: All dishes delivered to kitchen");

        return BT::NodeStatus::SUCCESS;
    }
};
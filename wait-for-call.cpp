#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

class WaitForCall : public BT::SyncActionNode, public rclcpp::Node
{
public:
    WaitForCall(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("wait_for_call_node") {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::vector<int>>("dishes"),
            BT::OutputPort<int>("dishes_quantity")
        };
    }

    BT::NodeStatus tick() override
    {
        int dishes_quantity;
        std::vector<int> dishes;

        RCLCPP_INFO(this->get_logger(), "Enter the number of rooms that need dishes collected:");
        std::cin >> dishes_quantity;

        RCLCPP_INFO(this->get_logger(), "Enter the room IDs (press Enter after each):");
        for (int i = 0; i < dishes_quantity; ++i)
        {
            int room_id;
            std::cout << "Room " << (i + 1) << ": ";
            std::cin >> room_id;
            dishes.push_back(room_id);
            RCLCPP_INFO(this->get_logger(), "[INFO] Room %d added to dishes list.", room_id);
        }

        setOutput("dishes", dishes);
        setOutput("dishes_quantity", dishes_quantity);

        RCLCPP_INFO(this->get_logger(), "[INFO] WaitForCall: Finished collecting calls: %d rooms collected", dishes_quantity);

        return BT::NodeStatus::SUCCESS;
    }
};
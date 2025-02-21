#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

class DeliverToTable : public BT::SyncActionNode, public rclcpp::Node
{
public:
    DeliverToTable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("deliver_to_table_node") {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(this->get_logger(), "Delivering meal to table...");
        return BT::NodeStatus::SUCCESS;
    }
};
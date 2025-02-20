#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class GoToRechargeBase : public BT::SyncActionNode
{
public:
    GoToRechargeBase(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("recharge_base_x", "Default X coordinate for recharge base"),
            BT::InputPort<double>("recharge_base_y", "Default Y coordinate for recharge base"),
            BT::InputPort<double>("recharge_base_yaw", "Default Yaw angle for recharge base")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("recharge_base_x", x) || !getInput("recharge_base_y", y) || !getInput("recharge_base_yaw", yaw))
        {
            std::cout << "[ERROR] GoToRechargeBase: Failed to get position inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToRechargeBase: Going to recharge base at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

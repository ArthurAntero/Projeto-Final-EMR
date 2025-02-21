#include <behaviortree_cpp_v3/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class GoToPose : public BT::SyncActionNode
{
public:
    GoToPose(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw))
        {
            std::cout << "[ERROR] GoToPose: Failed to get position and orientation inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToPose: Moving to position [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

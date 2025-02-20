#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class GoToKitchen : public BT::SyncActionNode
{
public:
    GoToKitchen(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("kitchen_x"),
            BT::InputPort<double>("kitchen_y"),
            BT::InputPort<double>("kitchen_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("kitchen_x", x) || !getInput("kitchen_y", y) || !getInput("kitchen_yaw", yaw))
        {
            std::cout << "[ERROR] GoToKitchen: Failed to get kitchen position inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToKitchen: Going to kitchen at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class GoToRoom : public BT::SyncActionNode
{
public:
    GoToRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("room_x"),
            BT::InputPort<double>("room_y"),
            BT::InputPort<double>("room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;
        if (!getInput("room_x", x) || !getInput("room_y", y) || !getInput("room_yaw", yaw))
        {
            std::cout << "[ERROR] GoToRoom: Failed to get room coordinates" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToRoom: Going to room at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

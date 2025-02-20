#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class GoToWaitingRoom : public BT::SyncActionNode
{
public:
    GoToWaitingRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("waiting_room_x"),
            BT::InputPort<double>("waiting_room_y"),
            BT::InputPort<double>("waiting_room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;
        if (!getInput("waiting_room_x", x) || !getInput("waiting_room_y", y) || !getInput("waiting_room_yaw", yaw))
        {
            std::cout << "[ERROR] GoToWaitingRoom: Failed to get waiting room coordinates" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToWaitingRoom: Going to waiting room at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

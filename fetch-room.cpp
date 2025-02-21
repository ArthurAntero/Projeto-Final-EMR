#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class FetchRoom : public BT::SyncActionNode
{
public:
    FetchRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, int>>("meals"),
            BT::InputPort<std::unordered_map<int, std::vector<double>>>("room_positions"),
            BT::OutputPort<double>("room_x"),
            BT::OutputPort<double>("room_y"),
            BT::OutputPort<double>("room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, int> meals;
        std::unordered_map<int, std::vector<double>> room_positions;

        if (!getInput("meals", meals))
        {
            std::cout << "[ERROR] FetchRoom: Failed to get meals from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("room_positions", room_positions))
        {
            std::cout << "[ERROR] FetchRoom: Failed to get room positions from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            std::cout << "[WARNING] FetchRoom: No meals available for delivery" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        int room_id = meals.begin()->second;

        if (room_positions.find(room_id) == room_positions.end())
        {
            std::cout << "[ERROR] FetchRoom: Room ID " << room_id << " not found in predefined positions" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::vector<double> coords = room_positions[room_id];
        setOutput("room_x", coords[0]);
        setOutput("room_y", coords[1]);
        setOutput("room_yaw", coords[2]);

        std::cout << "[INFO] FetchRoom: Fetching first available room " << room_id
                  << " with coordinates [x: " << coords[0] << ", y: " << coords[1]
                  << ", yaw: " << coords[2] << "]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
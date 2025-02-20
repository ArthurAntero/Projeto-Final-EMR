#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class FetchRoomForDishes : public BT::SyncActionNode
{
public:
    FetchRoomForDishes(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<int>>("dishes"),
            BT::InputPort<std::unordered_map<int, std::vector<double>>>("room_positions"),
            BT::OutputPort<double>("room_x"),
            BT::OutputPort<double>("room_y"),
            BT::OutputPort<double>("room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<int> dishes;
        std::unordered_map<int, std::vector<double>> room_positions;

        if (!getInput("dishes", dishes))
        {
            std::cout << "[ERROR] FetchRoomForDishes: Failed to get dishes from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("room_positions", room_positions))
        {
            std::cout << "[ERROR] FetchRoomForDishes: Failed to get room positions from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (dishes.empty())
        {
            std::cout << "[WARNING] FetchRoomForDishes: No dishes available for delivery" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        int room_id = dishes.front();

        if (room_positions.find(room_id) == room_positions.end())
        {
            std::cout << "[ERROR] FetchRoomForDishes: Room ID " << room_id << " not found in predefined positions" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::vector<double> coords = room_positions[room_id];
        setOutput("room_x", coords[0]);
        setOutput("room_y", coords[1]);
        setOutput("room_yaw", coords[2]);

        std::cout << "[INFO] FetchRoomForDishes: Fetching first available room " << room_id
                  << " with coordinates [x: " << coords[0] << ", y: " << coords[1]
                  << ", yaw: " << coords[2] << "]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

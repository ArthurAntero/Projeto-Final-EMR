#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class FetchRoom : public BT::SyncActionNode, public rclcpp::Node
{
public:
    FetchRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("fetch_room_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node FetchRoom initialized");
    }

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
            RCLCPP_ERROR(this->get_logger(), "Failed to get meals from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("room_positions", room_positions))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get room positions from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No meals available for delivery");
            return BT::NodeStatus::FAILURE;
        }

        int room_id = meals.begin()->second;

        if (room_positions.find(room_id) == room_positions.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Room ID %d not found in predefined positions", room_id);
            return BT::NodeStatus::FAILURE;
        }

        std::vector<double> coords = room_positions[room_id];
        setOutput("room_x", coords[0]);
        setOutput("room_y", coords[1]);
        setOutput("room_yaw", coords[2]);

        RCLCPP_INFO(this->get_logger(),
                    "Fetching first available room %d with coordinates [x: %.2f, y: %.2f, yaw: %.2f]",
                    room_id, coords[0], coords[1], coords[2]);

        return BT::NodeStatus::SUCCESS;
    }
};
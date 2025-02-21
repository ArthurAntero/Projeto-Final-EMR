#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>

class GotRightMeal : public BT::ConditionNode, public rclcpp::Node
{
public:
    GotRightMeal(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), rclcpp::Node("got_right_meal_node") {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, std::vector<double>>>("room_positions"),
            BT::InputPort<std::unordered_map<int, int>>("meals")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, std::vector<double>> room_positions;
        std::unordered_map<int, int> meals;

        if (!getInput("room_positions", room_positions))
        {
            RCLCPP_ERROR(this->get_logger(), "GotRightMeal: Failed to get room positions from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("meals", meals))
        {
            RCLCPP_ERROR(this->get_logger(), "GotRightMeal: Failed to get meals map from blackboard");
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            RCLCPP_WARN(this->get_logger(), "GotRightMeal: No meals available for delivery");
            return BT::NodeStatus::FAILURE;
        }

        auto it = meals.begin();
        if (it != meals.end())
        {
            int meal_id = it->first;
            int room_id = it->second;

            if (room_positions.find(room_id) == room_positions.end())
            {
                RCLCPP_ERROR(this->get_logger(), "GotRightMeal: Room ID %d not found in predefined positions", room_id);
                return BT::NodeStatus::FAILURE;
            }

            meals.erase(it);

            if (auto bb = config().blackboard)
            {
                bb->set("meals", meals);
            }

            RCLCPP_INFO(this->get_logger(), "GotRightMeal: Meal ID %d successfully delivered to room %d", meal_id, room_id);
        }

        return BT::NodeStatus::SUCCESS;
    }
};
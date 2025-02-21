#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>
#include <limits>

class GotRightMeal : public BT::ConditionNode
{
public:
    GotRightMeal(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::unordered_map<int, std::vector<double>>>("room_positions"),
            BT::InputPort<std::unordered_map<int, int>>("meals"),
            BT::OutputPort<std::unordered_map<int, int>>("meals")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, std::vector<double>> room_positions;
        std::unordered_map<int, int> meals;

        if (!getInput("room_positions", room_positions))
        {
            std::cerr << "[ERROR] GotRightMeal: Failed to get room positions from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("meals", meals))
        {
            std::cerr << "[ERROR] GotRightMeal: Failed to get meals map from blackboard" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (meals.empty())
        {
            std::cerr << "[WARNING] GotRightMeal: No meals available for delivery" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] Meals map content before deletion:" << std::endl;

        auto it = meals.begin();
        if (it != meals.end())
        {
            int meal_id = it->first;
            int room_id = it->second;

            if (room_positions.find(room_id) == room_positions.end())
            {
                std::cerr << "[ERROR] GotRightMeal: Room ID " << room_id << " not found in predefined positions" << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            meals.erase(it);

            if (auto bb = config().blackboard)
            {
                bb->set("meals", meals);
            }

            std::cout << "[INFO] GotRightMeal: Meal ID " << meal_id << " successfully delivered to room " << room_id << std::endl;
        }

        return BT::NodeStatus::SUCCESS;
    }
};

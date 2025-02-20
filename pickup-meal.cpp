#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>

class PickUpMeal : public BT::SyncActionNode, public rclcpp::Node
{
public:
    PickUpMeal(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("pick_up_meal") {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::unordered_map<int, int>>("meals"),
            BT::OutputPort<int>("meals_quantity")
        };
    }

    BT::NodeStatus tick() override
    {
        std::unordered_map<int, int> meals;
        int meals_quantity;

        std::cout << "Enter the number of meals to pick up: ";
        std::cin >> meals_quantity;

        for (int i = 0; i < meals_quantity; i++)
        {
            int meal_id = i;
            int room_id;

            std::cout << "Enter the room ID for meal " << meal_id + 1 << ": ";
            std::cin >> room_id;

            meals[meal_id] = room_id;
            RCLCPP_INFO(this->get_logger(), "PickUpMeal: Meal %d assigned to room %d", meal_id, room_id);
        }

        setOutput("meals", meals);
        setOutput("meals_quantity", meals_quantity);

        return BT::NodeStatus::SUCCESS;
    }
};

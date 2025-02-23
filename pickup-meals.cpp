#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class PickUpMeals : public BT::SyncActionNode
{
public:
    PickUpMeals(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::unordered_map<int, int>>("meals"),
            BT::OutputPort<int>("meals_quantity")
        };
    }

    BT::NodeStatus tick() override
    {
        int meals_quantity;
        std::unordered_map<int, int> meals;

        std::cout << "Enter the number of meals to pick up: ";
        std::cin >> meals_quantity;

        for (int i = 0; i < meals_quantity; i++)
        {
            int meal_id = i;
            int room_id;
            std::cout << "Enter the room ID for meal " << meal_id + 1 << ": ";
            std::cin >> room_id;
            meals[meal_id] = room_id;
            std::cout << "[INFO] PickUpMeals: Meal " << meal_id << " assigned to room " << room_id << std::endl;
        }

        setOutput("meals", meals);
        setOutput("meals_quantity", meals_quantity);
        return BT::NodeStatus::SUCCESS;
    }
};
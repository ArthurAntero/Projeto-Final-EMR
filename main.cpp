#include "deliver-dishes-to-kitchen.cpp"
#include "deliver-meal-to-table.cpp"
#include "fetch-room.cpp"
#include "go-to-pose.cpp"
#include "got-right-meal.cpp"
#include "is-battery-above-15.cpp"
#include "is-pickup-available.cpp"
#include "pickup-dish.cpp"
#include "pickup-meals.cpp"
#include "wait-for-call.cpp"
#include "wait-for-pickup.cpp"

class BTExecutor : public rclcpp::Node
{
public:
    BTExecutor() : Node("robot_behavior_tree") {}

    void create_behavior_tree()
    {
        rclcpp::Parameter str_param = this->get_parameter("bt");
        std::string tree_xml = str_param.as_string();

        RCLCPP_INFO(get_logger(), "Registering Nodes");
        
        BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<IsBatteryAbove15>(name, config, shared_from_this());
        };
        factory_.registerBuilder<IsBatteryAbove15>("IsBatteryAbove15", builder);

        builder = [=](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<GoToPose>(name, config, shared_from_this());
        };
        factory_.registerBuilder<GoToPose>("GoToPose", builder);

        RCLCPP_INFO(get_logger(), "Creating Tree %s", tree_xml.c_str());
        tree_ = factory_.createTreeFromFile(tree_xml);

        BT::BehaviorTreeFactory factory;

        factory_.registerNodeType<DeliverDishesToKitchen>("DeliverDishesToKitchen");
        factory_.registerNodeType<DeliverMealToTable>("DeliverMealToTable");
        factory_.registerNodeType<FetchRoom>("FetchRoom");
        factory_.registerNodeType<FetchRoom>("FetchRoom");
        factory_.registerNodeType<GotRightMeal>("GotRightMeal");
        factory_.registerNodeType<IsPickUpAvailable>("IsPickUpAvailable");
        factory_.registerNodeType<PickUpDish>("PickUpDish");
        factory_.registerNodeType<PickUpMeals>("PickUpMeals");
        factory_.registerNodeType<WaitForCall>("WaitForCall");
        factory_.registerNodeType<WaitForPickUp>("WaitForPickUp");
    
        auto tree = factory_.createTreeFromFile("../config/bt.xml");
    }

    void run()
    {
        while (rclcpp::ok())
        {
            tree_.rootBlackboard()->set("recharge_base_x", 0.0);
            tree_.rootBlackboard()->set("recharge_base_y", 0.0);
            tree_.rootBlackboard()->set("recharge_base_yaw", 0.0);
            tree_.rootBlackboard()->set("kitchen_x", 5.0);
            tree_.rootBlackboard()->set("kitchen_y", 5.0);
            tree_.rootBlackboard()->set("kitchen_yaw", 90.0);
            tree_.rootBlackboard()->set("waiting_room_x", 5.0);
            tree_.rootBlackboard()->set("waiting_room_y", 5.0);
            tree_.rootBlackboard()->set("waiting_room_yaw", 90.0);
            tree_.rootBlackboard()->set<std::unordered_map<int, std::vector<double>>>("room_positions", {{101, {1.0, 1.0, 90.0}}, {102, {2.0, 2.0, 90.0}}, {103, {3.0, 3.0, 90.0}}});
            tree_.rootBlackboard()->set<std::unordered_map<int, bool>>("available", {{101, true}, {102, false}, {103, true}});
            tree_.rootBlackboard()->set("wait_time_sec", 10);
            tree_.rootBlackboard()->set("pickup_time_sec", 10);

            auto result = tree_.tickRootWhileRunning();

            std::cout << "\nBT ended with result: " << result << std::endl;

            rclcpp::spin_some(shared_from_this());
        }
    }

private:
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<BTExecutor>();

    executor->create_behavior_tree();
    executor->run();

    rclcpp::shutdown();
    return 0;
}
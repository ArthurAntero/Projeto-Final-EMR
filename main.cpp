#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

#include "deliver-dish-to-kitchen.cpp"
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

int main(int argc, char **argv)
{

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<DeliverDishesToKitchen>("DeliverDishesToKitchen");
    factory.registerNodeType<DeliverMealToTable>("DeliverMealToTable");
    factory.registerNodeType<FetchRoom>("FetchRoom");
    factory.registerNodeType<GoToPose>("GoToPose");
    factory.registerNodeType<GotRightMeal>("GotRightMeal");
    factory.registerNodeType<IsBatteryAbove15>("IsBatteryAbove15");
    factory.registerNodeType<IsPickUpAvailable>("IsPickUpAvailable");
    factory.registerNodeType<PickUpDish>("PickUpDish");
    factory.registerNodeType<PickUpMeals>("PickUpMeals");
    factory.registerNodeType<WaitForCall>("WaitForCall");
    factory.registerNodeType<WaitForPickUp>("WaitForPickUp");


    auto tree = factory.createTreeFromFile("./bt.xml");



    while (true)
    {
        tree.rootBlackboard()->set("recharge_base_x", 0.0);
        tree.rootBlackboard()->set("recharge_base_y", 0.0);
        tree.rootBlackboard()->set("recharge_base_yaw", 0.0);
        tree.rootBlackboard()->set("kitchen_x", 5.0);
        tree.rootBlackboard()->set("kitchen_y", 5.0);
        tree.rootBlackboard()->set("kitchen_yaw", 90.0);
        tree.rootBlackboard()->set("waiting_room_x", 5.0);
        tree.rootBlackboard()->set("waiting_room_y", 5.0);
        tree.rootBlackboard()->set("waiting_room_yaw", 90.0);
        tree.rootBlackboard()->set<std::unordered_map<int, std::vector<double>>>("room_positions", {{101, {1.0, 1.0, 90.0}}, {102, {2.0, 2.0, 90.0}}, {103, {3.0, 3.0, 90.0}}});
        tree.rootBlackboard()->set<std::unordered_map<int, bool>>("available", {{101, true}, {102, false}, {103, true}});
        tree.rootBlackboard()->set("pickup_time_sec", 10);

        auto result = tree.tickRootWhileRunning();

        std::cout << "\nBT ended with result: " << result << std::endl;
    }

    return 0;
}

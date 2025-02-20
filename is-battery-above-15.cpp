#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class IsBatteryAbove15 : public BT::ConditionNode
{
public:
    IsBatteryAbove15(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<double>("battery_level")};
    }

    BT::NodeStatus tick() override
    {

        double battery_level;
        std::cout << "Enter current battery level: ";
        std::cin >> battery_level;

        setOutput("battery_level", battery_level);

        
        if (battery_level > 15.0)
        {
            std::cout << "[INFO] IsBatteryAbove15: Battery level is above 15%" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[WARNING] IsBatteryAbove15: Battery level is below or equal to 15%" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

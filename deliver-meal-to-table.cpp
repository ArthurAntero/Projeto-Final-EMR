#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class DeliverMealToTable : public BT::SyncActionNode
{
public:
    DeliverMealToTable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::cout << "[INFO] DeliverMealToTable: Delivering meal to table..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
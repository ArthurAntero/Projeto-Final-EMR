#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>

class DeliverToTable : public BT::SyncActionNode
{
public:
    DeliverToTable(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std::cout << "[INFO] DeliverToTable: Delivering meal to table..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

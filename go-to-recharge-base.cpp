#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <chrono>
#include <thread>

class GoToRechargeBase : public BT::SyncActionNode
{
public:
    explicit GoToRechargeBase(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
    {
        publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("recharge_base_x"),
            BT::InputPort<double>("recharge_base_y"),
            BT::InputPort<double>("recharge_base_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("recharge_base_x", x) || !getInput("recharge_base_y", y) || !getInput("recharge_base_yaw", yaw))
        {
            std::cout << "[ERROR] GoToRechargeBase: Failed to get position inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToRechargeBase: Going to recharge base at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.z = yaw;

        publisher_->publish(pose_msg);

        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "[INFO] GoToRechargeBase: Recharge complete." << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

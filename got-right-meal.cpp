#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>

class GoToRoom : public BT::SyncActionNode
{
public:
    explicit GoToRoom(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
    {
        publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("room_x"),
            BT::InputPort<double>("room_y"),
            BT::InputPort<double>("room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("room_x", x) || !getInput("room_y", y) || !getInput("room_yaw", yaw))
        {
            std::cout << "[ERROR] GoToRoom: Failed to get room position inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] GoToRoom: Going to room at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.z = yaw;

        publisher_->publish(pose_msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>

class GoToWaitingRoom : public BT::SyncActionNode, public rclcpp::Node
{
public:
    GoToWaitingRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_waiting_room_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("waiting_room_x"),
            BT::InputPort<double>("waiting_room_y"),
            BT::InputPort<double>("waiting_room_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;
        if (!getInput("waiting_room_x", x) || !getInput("waiting_room_y", y) || !getInput("waiting_room_yaw", yaw))
        {
            RCLCPP_ERROR(this->get_logger(), "GoToWaitingRoom: Failed to get waiting room coordinates");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.z = sin(yaw / 2.0);
        goal_pose.pose.orientation.w = cos(yaw / 2.0);

        publisher_->publish(goal_pose);

        RCLCPP_INFO(this->get_logger(), "GoToWaitingRoom: Published goal to waiting room at [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw * 0.5);
        return q;
    }
};
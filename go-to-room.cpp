#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <thread>

class GoToRoom : public BT::SyncActionNode, public rclcpp::Node
{
public:
    GoToRoom(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_room_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
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
            RCLCPP_ERROR(this->get_logger(), "[ERROR] GoToRoom: Failed to get room coordinates");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "[INFO] GoToRoom: Going to room at [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);

        // Criação da mensagem de pose
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.position.z = 0.0;
        
        // Conversão do ângulo de yaw para um quaternion
        goal_msg.pose.orientation = yawToQuaternion(yaw);

        // Publica a mensagem no tópico goal_pose
        publisher_->publish(goal_msg);

        RCLCPP_INFO(this->get_logger(), "[INFO] GoToRoom: Goal sent successfully!");
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
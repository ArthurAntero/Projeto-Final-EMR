#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>

class GoToPose : public BT::SyncActionNode
{
public:
    explicit GoToPose(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
    {
        // Criar o publisher de movimento para o tópico "goal_pose"
        publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        // Obter as entradas para x, y e yaw
        if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw))
        {
            std::cout << "[ERROR] GoToPose: Failed to get position inputs" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Informar a posição e orientação para o console
        std::cout << "[INFO] GoToPose: Going to position [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        // Criar a mensagem PoseStamped com as coordenadas e orientação recebidas
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";  // Ou "base_link", dependendo da sua configuração
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.z = yaw;  // Pode usar apenas yaw para orientação 2D (para simplificação)

        // Publicar a mensagem no tópico "goal_pose"
        publisher_->publish(pose_msg);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;  // Ponteiro para o nó ROS 2
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;  // Publisher para o movimento
};

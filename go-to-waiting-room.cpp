#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <chrono>
#include <thread>

class GoToWaitingRoom : public BT::SyncActionNode
{
public:
    // Construtor agora recebe o ponteiro para o nó ROS 2
    GoToWaitingRoom(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
        : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
    {
        // Criar o publisher de movimento
        publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }

    // Definindo as portas de entrada (coordenadas)
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("waiting_room_x"),
            BT::InputPort<double>("waiting_room_y"),
            BT::InputPort<double>("waiting_room_yaw")
        };
    }

    // Função tick que é chamada a cada ciclo da árvore de comportamento
    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        // Obtendo as entradas do Blackboard
        if (!getInput("waiting_room_x", x) || !getInput("waiting_room_y", y) || !getInput("waiting_room_yaw", yaw))
        {
            std::cout << "[ERROR] GoToWaitingRoom: Failed to get waiting room coordinates" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Imprimindo as coordenadas para debug
        std::cout << "[INFO] GoToWaitingRoom: Going to waiting room at [x: " << x << ", y: " << y << ", yaw: " << yaw << "]" << std::endl;

        // Criar a mensagem PoseStamped para mover o robô
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";  // ou o frame adequado, como "base_link"
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.z = yaw;  // Para simplificação, você pode usar yaw para orientação Z

        // Publicar a mensagem para mover o robô até a sala
        publisher_->publish(pose_msg);

        return BT::NodeStatus::SUCCESS;  // Retorna sucesso após movimento (ou outro critério)
    }

private:
    rclcpp::Node::SharedPtr node_ptr_;  // Ponteiro para o nó ROS2
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};
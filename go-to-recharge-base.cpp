#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>

class GoToRechargeBase : public BT::SyncActionNode, public rclcpp::Node
{
public:
    GoToRechargeBase(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_recharge_base")
    {
        // Inicializa o publisher para o tópico goal_pose
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
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

        // Obtém as posições do blackboard
        if (!getInput("recharge_base_x", x) || !getInput("recharge_base_y", y) || !getInput("recharge_base_yaw", yaw))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get position inputs");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(), "Going to recharge base at [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);

        // Cria a mensagem PoseStamped
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->get_clock()->now();
        goal_msg.header.frame_id = "map"; // Sistema de coordenadas do mapa

        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.position.z = 0.0;

        // Converte yaw para quaternion
        goal_msg.pose.orientation = yawToQuaternion(yaw);

        // Publica a mensagem no tópico /goal_pose
        publisher_->publish(goal_msg);

        std::cout << "[INFO] Recharging..." << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

private:
    // Função para converter Yaw em um quaternion
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw * 0.5);
        return q;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};
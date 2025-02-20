#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <thread>

class GoToRechargeBase : public BT::SyncActionNode, public rclcpp::Node
{
public:
    GoToRechargeBase(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_recharge_base")
    {
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

        if (!getInput("recharge_base_x", x) || !getInput("recharge_base_y", y) || !getInput("recharge_base_yaw", yaw))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get position inputs");
            return BT::NodeStatus::FAILURE;
        }

        // Criando a mensagem para enviar ao /goal_pose
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.orientation.w = 1.0; // Supondo orientação padrão

        // Publicando a mensagem para mover o robô
        publisher_->publish(goal_msg);

        RCLCPP_INFO(this->get_logger(), "Published goal to /goal_pose: x=%.2f, y=%.2f", x, y);

        // Simula um tempo de recarga
        std::cout << "[INFO] Recharging..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

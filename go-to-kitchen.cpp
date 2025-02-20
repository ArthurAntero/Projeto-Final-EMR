#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GoToKitchen : public BT::SyncActionNode, public rclcpp::Node
{
public:
    GoToKitchen(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_kitchen")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("kitchen_x"),
            BT::InputPort<double>("kitchen_y"),
            BT::InputPort<double>("kitchen_yaw")
        };
    }

    BT::NodeStatus tick() override
    {
        double x, y, yaw;

        if (!getInput("kitchen_x", x) || !getInput("kitchen_y", y) || !getInput("kitchen_yaw", yaw))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get kitchen position inputs");
            return BT::NodeStatus::FAILURE;
        }

        // Criando a mensagem para o tópico /goal_pose
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.orientation.w = 1.0; // Assumindo orientação padrão

        // Publicando no tópico
        publisher_->publish(goal_msg);

        RCLCPP_INFO(this->get_logger(), "Published goal to /goal_pose: x=%.2f, y=%.2f", x, y);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>

class GoToRechargeBase : public BT::SyncActionNode, public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToRechargeBase(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), rclcpp::Node("go_to_recharge_base")
    {
        // Criando cliente de ação para enviar o goal
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }
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

        RCLCPP_INFO(this->get_logger(), "Going to recharge base at [x: %.2f, y: %.2f, yaw: %.2f]", x, y, yaw);

        // Criando mensagem de navegação
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0; // Supondo orientação padrão

        // Enviando goal para o Nav2
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [](const GoalHandleNav::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                std::cout << "[INFO] Reached recharge base successfully!" << std::endl;
            }
            else
            {
                std::cout << "[ERROR] Failed to reach recharge base!" << std::endl;
            }
        };

        auto goal_handle_future = this->action_client_->async_send_goal(goal_msg, send_goal_options);

        // Aguarda o resultado antes de continuar
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to recharge base");
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[INFO] Recharging..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Simula recarga

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

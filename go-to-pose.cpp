#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");
  done_flag_ = false;
}


BT::NodeStatus GoToPose::onStart()
{
  auto goal_msg = NavigateToPose::Goal();
  auto x_in = getInput<double>("x");
  auto y_in = getInput<double>("y");
  auto yaw_in = getInput<double>("yaw");
  double x = x_in.value(), y = y_in.value(), yaw = yaw_in.value();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  done_flag_ = false;
  for(int i=0; i<5; i++){
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    sleep(1);
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_){
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal reached\n");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    done_flag_ = true;
}
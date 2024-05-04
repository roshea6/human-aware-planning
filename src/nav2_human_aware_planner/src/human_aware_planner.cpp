/*********************************************************************
 * Author: Ryan O'Shea
 * Based off HumanAware planner tutorial from
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_human_aware_planner/human_aware_planner.hpp"

namespace nav2_human_aware_planner
{

void HumanAware::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // START Subscription to path topic
  path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/custom_path", rclcpp::SensorDataQoS(),
      std::bind(&HumanAware::pathCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "StraighLine: subscribed to "
              "topic %s",
              path_sub_->get_topic_name());

  start_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("robot_start_pose", 10);
  goal_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("robot_goal_pose", 10);

  path_received = false;

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void HumanAware::pathCallback(
      const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_message_mutex_.lock();
    path_list_ = *msg;
    // std::printf("SocialLayer. people received: %i", (int)people_list_.people.size());
    path_message_mutex_.unlock();
    RCLCPP_INFO(node_->get_logger(),
              "PATH RECEIVED BY PLANNER NODE!");
    path_received = true;
  }

void HumanAware::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void HumanAware::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void HumanAware::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path HumanAware::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  // Republish the start and goal poses so the other script can use them
  start_pub->publish(start);
  goal_pub->publish(goal);

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  
  // Add the start node to the list
  geometry_msgs::msg::PoseStamped start_pose = start;
  start_pose.header.stamp = node_->now();
  start_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(start_pose);

  // If the path hasn't been received yet just publish the blank one
  if(path_received == false)
  {
    RCLCPP_INFO(node_->get_logger(),
              "Path not yet received. Please run planner node");
    return global_path;
  }


  global_path.poses = path_list_.poses;

  // // calculating the number of loops for current value of interpolation_resolution_
  // int total_number_of_loop = std::hypot(
  //   goal.pose.position.x - start.pose.position.x,
  //   goal.pose.position.y - start.pose.position.y) /
  //   interpolation_resolution_;
  // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // for (int i = 0; i < total_number_of_loop; ++i) {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.pose.position.x = start.pose.position.x + x_increment * i;
  //   pose.pose.position.y = start.pose.position.y + y_increment * i;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }

  // geometry_msgs::msg::PoseStamped goal_pose = goal;
  // goal_pose.header.stamp = node_->now();
  // goal_pose.header.frame_id = global_frame_;
  // global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace nav2_human_aware_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_human_aware_planner::HumanAware, nav2_core::GlobalPlanner)

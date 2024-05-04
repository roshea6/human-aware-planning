/*********************************************************************
 * Author: Ryan O'Shea
 * Based off HumanAware planner tutorial from
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#ifndef NAV2_HUMAN_AWARE_PLANNER__HUMAN_AWARE_PLANNER_HPP_
#define NAV2_HUMAN_AWARE_PLANNER__HUMAN_AWARE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_human_aware_planner
{

class HumanAware : public nav2_core::GlobalPlanner
{
public:
  HumanAware() = default;
  ~HumanAware() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;

  bool path_received;

  nav_msgs::msg::Path path_list_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  std::mutex path_message_mutex_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
};

}  // namespace nav2_human_aware_planner

#endif  // NAV2_HUMAN_AWARE_PLANNER__HUMAN_AWARE_PLANNER_HPP_

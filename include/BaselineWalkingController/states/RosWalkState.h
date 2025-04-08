#pragma once

#include <BaselineWalkingController/State.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/subscription_base.hpp>

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace BWC
{
/** \brief FSM state to send footstep via ROS topic. */
struct RosWalkState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Set goal pose from ROS topic. */
  void setGoal();

  /** \brief Walk to the goal pose. */
  void walkToGoal();

  /** \brief ROS callback of pose topic. */
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & poseMsg);

protected:
  //! List of frame names representing world coordinates
  const std::vector<std::string> worldFrameNames_ = {"world", "map", "robot_map"};

  //! Goal foot midpose (x [m], y [m], theta [rad])
  Eigen::Vector3d goalFootMidTrans_ = {0, 0, 0};

  //! Goal pose offset (x [m], y [m], theta [rad])
  Eigen::Vector3d goalOffset_ = {0, 0, 0};

  //! ROS variables
  //! @{
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Executor::SharedPtr executor_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
  geometry_msgs::msg::PoseStamped::SharedPtr poseMsg_;
  //! @}
};
} // namespace BWC

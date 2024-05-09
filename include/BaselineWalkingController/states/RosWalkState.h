#pragma once

#include <BaselineWalkingController/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

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
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseMsg);

protected:
  //! List of frame names representing world coordinates
  const std::vector<std::string> worldFrameNames_ = {"world", "map", "robot_map"};

  //! Goal foot midpose (x [m], y [m], theta [rad])
  Eigen::Vector3d goalFootMidTrans_ = {0, 0, 0};

  //! Goal pose offset (x [m], y [m], theta [rad])
  Eigen::Vector3d goalOffset_ = {0, 0, 0};

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber poseSub_;
  std::shared_ptr<geometry_msgs::PoseStamped> poseMsg_;
  //! @}
};
} // namespace BWC

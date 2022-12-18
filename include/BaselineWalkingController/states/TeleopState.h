#pragma once

#include <BaselineWalkingController/State.h>

#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace BWC
{
/** \brief FSM state to walk with teleoperation. */
struct TeleopState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief ROS callback of twist topic. */
  void twistCallback(const geometry_msgs::Twist::ConstPtr & twistMsg);

protected:
  //! Relative target velocity of foot midpose (x [m/s], y [m/s], theta [rad/s])
  Eigen::Vector3d targetVel_ = Eigen::Vector3d::Zero();

  //! Scale to convert twist message to target velocity (x, y, theta)
  Eigen::Vector3d velScale_ = Eigen::Vector3d::Ones();

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber twistSub_;
  //! @}
};
} // namespace BWC

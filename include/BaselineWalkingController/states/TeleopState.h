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
  /** \brief Start teleoperation. */
  void startTeleop();

  /** \brief End teleoperation. */
  void endTeleop();

  /** \brief ROS callback of twist topic. */
  void twistCallback(const geometry_msgs::Twist::ConstPtr & twistMsg);

protected:
  //! Whether to run teleoperation
  bool runTeleop_ = false;

  //! Whether starting teleoperation is triggered
  bool startTriggered_ = false;

  //! Whether ending teleoperation is triggered
  bool endTriggered_ = false;

  //! Command footstep queue size
  int footstepQueueSize_ = 3;

  //! Velocity scale (x, y, theta)
  Eigen::Vector3d velScale_ = Eigen::Vector3d(0.3, 0.2, mc_rtc::constants::toRad(15));

  //! Limit of foot midpose transformation for one footstep (x [m], y [m], theta [rad])
  Eigen::Vector3d deltaTransLimit_ = Eigen::Vector3d(0.15, 0.1, mc_rtc::constants::toRad(15));

  //! Target velocity (x [m/s], y [m/s], theta [rad/s])
  Eigen::Vector3d targetVel_ = Eigen::Vector3d::Zero();

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber twistSub_;
  //! @}
};
} // namespace BWC

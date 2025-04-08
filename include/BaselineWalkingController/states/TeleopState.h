#pragma once

#include <BaselineWalkingController/State.h>

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

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
  void twistCallback(const geometry_msgs::msg::Twist::ConstSharedPtr & twistMsg);

protected:
  //! Relative target velocity of foot midpose (x [m/s], y [m/s], theta [rad/s])
  Eigen::Vector3d targetVel_ = Eigen::Vector3d::Zero();

  //! Scale to convert twist message to target velocity (x, y, theta)
  Eigen::Vector3d velScale_ = Eigen::Vector3d::Ones();

  //! ROS variables
  //! @{
  rclcpp::Node::SharedPtr nh_;
  
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub_;
  //! @}
};
} // namespace BWC

#pragma once

#include <BaselineWalkingController/State.h>

#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace BWC
{
/** \brief FSM state to walk with step by step. */
struct StepState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
};
} // namespace BWC

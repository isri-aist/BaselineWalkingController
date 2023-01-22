#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to initialize. */
struct InitialState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Phase
  int phase_ = 0;

  //! Function to interpolate task stiffness
  std::shared_ptr<TrajColl::CubicInterpolator<double>> stiffnessRatioFunc_;

  //! Stiffness of CoM task
  Eigen::Vector3d comTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of base link orientation task
  Eigen::Vector3d baseOriTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of foot tasks
  sva::MotionVecd footTasksStiffness_ = sva::MotionVecd::Zero();
};
} // namespace BWC

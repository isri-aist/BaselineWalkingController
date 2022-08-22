#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to walk with footstep planner. */
struct FootstepPlannerState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Whether planning and walking is triggered
  bool triggered_ = false;

  //! Goal foot midpose (x [m], y [m], theta [rad])
  std::array<double, 3> goalFootMidpose_ = std::array<double, 3>{0, 0, 0};
};
} // namespace BWC

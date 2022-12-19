#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to send footstep from configuration. */
struct ConfigWalkState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! End time of velocity mode [sec]
  double velModeEndTime_ = 0.0;
};
} // namespace BWC

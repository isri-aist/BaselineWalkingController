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
};
} // namespace BWC

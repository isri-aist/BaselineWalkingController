#pragma once

#include <mc_control/fsm/State.h>

namespace BWC
{
class BaselineWalkingController;

/** \brief FSM State with utility functions. */
struct State : mc_control::fsm::State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & _ctl) override;

protected:
  /** \brief Const accessor to the controller. */
  inline const BaselineWalkingController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline BaselineWalkingController & ctl()
  {
    return *ctlPtr_;
  }

protected:
  //! Pointer to controller
  BaselineWalkingController * ctlPtr_ = nullptr;
};
} // namespace BWC

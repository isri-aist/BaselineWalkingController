#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to send footstep from GUI. */
struct GuiWalkState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  //! Entry keys of GUI form
  const std::unordered_map<std::string, std::string> walkConfigKeys_ = {{"x", "goal x [m]"},
                                                                        {"y", "goal y [m]"},
                                                                        {"theta", "goal theta [deg]"},
                                                                        {"last", "number of last footstep"}};
};
} // namespace BWC

#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to send walking command from GUI. */
struct CommandState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Send a walking command.
      \param goalTrans goal transformation (x [m], y [m], theta [rad])
      \param lastFootstepNum number of last footstep
   */
  void sendWalkingCommand(const Eigen::Vector3d & goalTrans, int lastFootstepNum);

protected:
  //! Limit of foot midpose transformation for one footstep (x [m], y [m], theta [rad])
  Eigen::Vector3d deltaTransLimit_ = Eigen::Vector3d(0.15, 0.1, mc_rtc::constants::toRad(15));

  //! Form entry keys of walking command
  const std::unordered_map<std::string, std::string> walkConfigKeys_ = {{"x", "goal x [m]"},
                                                                        {"y", "goal y [m]"},
                                                                        {"theta", "goal theta [deg]"},
                                                                        {"last", "number of last footstep"}};
};
} // namespace BWC
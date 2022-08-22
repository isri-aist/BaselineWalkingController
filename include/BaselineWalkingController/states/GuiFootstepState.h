#pragma once

#include <BaselineWalkingController/State.h>

namespace BWC
{
/** \brief FSM state to send footstep from GUI. */
struct GuiFootstepState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Send footstep list.
      \param goalTrans goal transformation (x [m], y [m], theta [rad])
      \param lastFootstepNum number of last footstep
   */
  void sendFootstepList(const Eigen::Vector3d & goalTrans, int lastFootstepNum);

protected:
  //! Limit of foot midpose transformation for one footstep (x [m], y [m], theta [rad])
  Eigen::Vector3d deltaTransLimit_ = Eigen::Vector3d(0.15, 0.1, mc_rtc::constants::toRad(15));

  //! Entry keys of GUI form
  const std::unordered_map<std::string, std::string> walkConfigKeys_ = {{"x", "goal x [m]"},
                                                                        {"y", "goal y [m]"},
                                                                        {"theta", "goal theta [deg]"},
                                                                        {"last", "number of last footstep"}};
};
} // namespace BWC

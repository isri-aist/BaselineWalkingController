#include <mc_rtc/gui/Form.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/GuiWalkState.h>

using namespace BWC;

void GuiWalkState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "GuiWalk"},
                          mc_rtc::gui::Button("StopWalking", [this]() { ctl().footManager_->clearFootstepQueue(); }),
                          mc_rtc::gui::Form(
                              "Walk",
                              [this](const mc_rtc::Configuration & config) {
                                ctl().footManager_->walkToRelativePose(
                                    Eigen::Vector3d(config(walkConfigKeys_.at("x")), config(walkConfigKeys_.at("y")),
                                                    mc_rtc::constants::toRad(config(walkConfigKeys_.at("theta")))),
                                    config(walkConfigKeys_.at("last")));
                              },
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("x"), true, 0.0),
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("y"), true, 0.0),
                              mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("theta"), true, 0.0),
                              mc_rtc::gui::FormIntegerInput(walkConfigKeys_.at("last"), true, 0)));

  output("OK");
}

bool GuiWalkState::run(mc_control::fsm::Controller &)
{
  return false;
}

void GuiWalkState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "GuiWalk"});
}

EXPORT_SINGLE_STATE("BWC::GuiWalk", GuiWalkState)

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/ros.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/StepState.h>

using namespace BWC;

void StepState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);
  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "Step by Step"},
                          mc_rtc::gui::Button("StartStepState", [this]() { ctl().footManager_->startVelMode(); }));
  output("OK");
}

bool StepState::run(mc_control::fsm::Controller &)
{
  // Update GUI
  bool stepMode = ctl().footManager_->velModeEnabled();
  if(stepMode && ctl().gui()->hasElement({ctl().name(), "Step by Step"}, "StartStepState"))
  {
    ctl().gui()->addElement({ctl().name(), "Step by Step"},
                            mc_rtc::gui::Button("NextStatus", [this]() { ctl().footManager_->endVelMode(); }));
    ctl().gui()->addElement({ctl().name(), "Step by Step"},
                            mc_rtc::gui::Button("PreviousStatus", [this]() { ctl().footManager_->endVelMode(); }));
    ctl().gui()->removeElement({ctl().name(), "Step by Step"}, "StartStepState");
  }
//   else if(!stepMode && ctl().gui()->hasElement({ctl().name(), "Step by Step"}, "EndTeleop"))
//   {
//     ctl().gui()->addElement({ctl().name(), "Step by Step"},
//                             mc_rtc::gui::Button("StartTeleop", [this]() { ctl().footManager_->startVelMode(); }));
//     ctl().gui()->removeElement({ctl().name(), "Step by Step"}, "EndTeleop");
//   }

  return false;
}

void StepState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "Step by Step"});
}



EXPORT_SINGLE_STATE("BWC::Step", StepState)

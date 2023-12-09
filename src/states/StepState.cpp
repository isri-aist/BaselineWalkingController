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
                          mc_rtc::gui::Button("StartStepState", [this]() { ctl().footManager_->startStepMode(); }));
                          // mc_rtc::gui::Button("StartStepState", [this]() { ctl().footManager_->startVelMode(); }));
  output("OK");
}

bool StepState::run(mc_control::fsm::Controller &)
{
  // Update GUI
  bool stepMode = ctl().footManager_->stepModeEnabled();
  int stepCount = ctl().footManager_->stepCount();
  if(stepMode && ctl().gui()->hasElement({ctl().name(), "Step by Step"}, "StartStepState"))
  {
    ctl().gui()->addElement({ctl().name(), "Step by Step"},
                            mc_rtc::gui::Button("NextStatus", [this]() { ctl().footManager_->nextStepMode(); }));
    ctl().gui()->addElement({ctl().name(), "Step by Step"},
                            mc_rtc::gui::Button("Stamp", [this]() { ctl().footManager_->stampStepMode(); }));
    ctl().gui()->removeElement({ctl().name(), "Step by Step"}, "StartStepState");
  }

  if(stepMode && stepCount > 0 && 
    ctl().gui()->hasElement({ctl().name(), "Step by Step"}, "NextStatus") && 
    !ctl().gui() -> hasElement({ctl().name(), "Step by Step"}, "PreviousStatus"))
  {
    ctl().gui()->addElement({ctl().name(), "Step by Step"},
                        mc_rtc::gui::Button("PreviousStatus", [this]() { ctl().footManager_->previousStepMode(); }));
  }

  if(stepMode && stepCount == 0 && 
    ctl().gui()->hasElement({ctl().name(), "Step by Step"}, "NextStatus") && 
    ctl().gui() -> hasElement({ctl().name(), "Step by Step"}, "PreviousStatus"))
  {
    ctl().gui()->removeElement({ctl().name(), "Step by Step"},"PreviousStatus");
  }

  return false;
}

void StepState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "Step by Step"});
}



EXPORT_SINGLE_STATE("BWC::Step", StepState)

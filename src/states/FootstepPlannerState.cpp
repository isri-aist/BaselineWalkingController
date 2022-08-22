#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/XYTheta.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/FootstepPlannerState.h>

using namespace BWC;

void FootstepPlannerState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup GUI
  ctl().gui()->addElement(
      {"BWC", "FootstepPlanner"}, mc_rtc::gui::Button("PlanAndWalk", [this]() { triggered_ = true; }),
      mc_rtc::gui::XYTheta(
          "GoalPose", [this]() -> std::array<double, 3> { return goalFootMidpose_; },
          [this](const std::array<double, 4> & goalFootMidpose) {
            return std::copy(goalFootMidpose.begin(), goalFootMidpose.begin() + 3, goalFootMidpose_.begin());
          }));

  output("OK");
}

bool FootstepPlannerState::run(mc_control::fsm::Controller &)
{
  if(triggered_)
  {
    triggered_ = false;

    if(ctl().footManager_->footstepQueue().size() > 0)
    {
      mc_rtc::log::error(
          "[FootstepPlannerState] Planning and walking can be started only when the footstep queue is empty: {}",
          ctl().footManager_->footstepQueue().size());
    }
    else
    {
      // TODO
      mc_rtc::log::success("goalFootMidpose: [{}, {}, {}]", goalFootMidpose_[0], goalFootMidpose_[1],
                           goalFootMidpose_[2]);
    }
  }

  return false;
}

void FootstepPlannerState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({"BWC", "FootstepPlanner"});
}

EXPORT_SINGLE_STATE("BWC::FootstepPlanner", FootstepPlannerState)

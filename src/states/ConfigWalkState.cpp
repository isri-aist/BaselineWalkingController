#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/ConfigWalkState.h>

using namespace BWC;

void ConfigWalkState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Set footstep
  if(config_.has("configs") && config_("configs").has("footstepList"))
  {
    Foot foot = Foot::Left;
    double startTime = ctl().t();

    for(const auto & footstepConfig : config_("configs")("footstepList"))
    {
      if(footstepConfig.has("foot"))
      {
        foot = strToFoot(footstepConfig("foot"));
      }
      if(footstepConfig.has("startTime"))
      {
        startTime = ctl().t() + static_cast<double>(footstepConfig("startTime"));
      }
      const auto & footstep = ctl().footManager_->makeFootstep(foot, footstepConfig("footMidpose"), startTime,
                                                               (footstepConfig("config", mc_rtc::Configuration())));
      ctl().footManager_->appendFootstep(footstep);

      foot = opposite(foot);
      startTime = footstep.transitEndTime;
    }
  }

  output("OK");
}

bool ConfigWalkState::run(mc_control::fsm::Controller &)
{
  return ctl().footManager_->footstepQueue().empty();
}

void ConfigWalkState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::ConfigWalk", ConfigWalkState)

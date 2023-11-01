#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
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
      const auto & footstep = ctl().footManager_->makeFootstep(
          foot, footstepConfig("footMidpose"), startTime, footstepConfig("swingTrajConfig", mc_rtc::Configuration()));
      ctl().footManager_->appendFootstep(footstep);

      foot = opposite(foot);
      startTime = footstep.transitEndTime;
    }
  }
  else if(config_.has("configs") && config_("configs").has("velocityMode"))
  {
    ctl().footManager_->startVelMode();
    ctl().footManager_->setRelativeVel(config_("configs")("velocityMode")("velocity"));
    velModeEndTime_ = ctl().t() + static_cast<double>(config_("configs")("velocityMode")("duration"));
  }
  else if(config_.has("configs") && config_("configs").has("footMidpose"))
  {
    auto convertDegToRad = [](const Eigen::Vector3d & trans) -> Eigen::Vector3d {
      return Eigen::Vector3d(trans.x(), trans.y(), mc_rtc::constants::toRad(trans.z()));
    };
    Eigen::Vector3d targetTrans = convertDegToRad(config_("configs")("footMidpose")("target"));
    std::vector<Eigen::Vector3d> waypointTransList = {};
    if(config_("configs")("footMidpose").has("waypointList"))
    {
      for(const Eigen::Vector3d & waypointTrans : config_("configs")("footMidpose")("waypointList"))
      {
        waypointTransList.push_back(convertDegToRad(waypointTrans));
      }
    }
    ctl().footManager_->walkToRelativePose(targetTrans, 0, waypointTransList);
  }

  // Set reference CoM Z position
  if(config_.has("configs") && config_("configs").has("refComZList"))
  {
    for(const auto & refComZConfig : config_("configs")("refComZList"))
    {
      ctl().centroidalManager_->setRefComZ(refComZConfig("refComZ"),
                                           ctl().t() + static_cast<double>(refComZConfig("startTime")),
                                           refComZConfig("interpDuration"));
    }
  }

  output("OK");
}

bool ConfigWalkState::run(mc_control::fsm::Controller &)
{
  if(config_.has("configs") && config_("configs").has("footstepList"))
  {
    return ctl().footManager_->footstepQueue().empty();
  }
  else if(config_.has("configs") && config_("configs").has("velocityMode"))
  {
    if(ctl().t() > velModeEndTime_ && ctl().footManager_->velModeEnabled())
    {
      ctl().footManager_->endVelMode();
    }
    return !ctl().footManager_->velModeEnabled() && ctl().footManager_->footstepQueue().empty();
  }

  return true;
}

void ConfigWalkState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("BWC::ConfigWalk", ConfigWalkState)

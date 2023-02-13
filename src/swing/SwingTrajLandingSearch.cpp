#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <BaselineWalkingController/swing/SwingTrajLandingSearch.h>

using namespace BWC;

void SwingTrajLandingSearch::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("withdrawOffset", withdrawOffset);
  mcRtcConfig("preApproachDurationRatio", preApproachDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("approachOffset", approachOffset);
}

void SwingTrajLandingSearch::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajLandingSearch::addConfigToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.addElement(
      category,
      mc_rtc::gui::NumberInput(
          "withdrawDurationRatio", []() { return defaultConfig_.withdrawDurationRatio; },
          [](double v) { defaultConfig_.withdrawDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "withdrawOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.withdrawOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.withdrawOffset = v; }),
      mc_rtc::gui::NumberInput(
          "preApproachDurationRatio", []() { return defaultConfig_.preApproachDurationRatio; },
          [](double v) { defaultConfig_.preApproachDurationRatio = v; }),
      mc_rtc::gui::NumberInput(
          "approachDurationRatio", []() { return defaultConfig_.approachDurationRatio; },
          [](double v) { defaultConfig_.approachDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "approachOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.approachOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.approachOffset = v; }));
}

void SwingTrajLandingSearch::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                 const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajLandingSearch::SwingTrajLandingSearch(const sva::PTransformd & startPose,
                                               const sva::PTransformd & endPose,
                                               double startTime,
                                               double endTime,
                                               const TaskGain & taskGain,
                                               const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, endPose, startTime, endTime, taskGain, mcRtcConfig)
{
  config_.load(mcRtcConfig);

  double withdrawTime = (1 - config_.withdrawDurationRatio) * startTime_ + config_.withdrawDurationRatio * endTime_;
  double preApproachDurationRatioTotal = config_.preApproachDurationRatio + config_.approachDurationRatio;
  double preApproachTime = preApproachDurationRatioTotal * startTime_ + (1 - preApproachDurationRatioTotal) * endTime_;
  double approachTime = config_.approachDurationRatio * startTime_ + (1 - config_.approachDurationRatio) * endTime_;

  waypointPoseList_ = {{startTime_, startPose_},
                       {withdrawTime, sva::PTransformd(config_.withdrawOffset) * startPose_},
                       {preApproachTime, sva::PTransformd(config_.approachOffset) * endPose_},
                       {approachTime, sva::PTransformd(config_.approachOffset) * endPose_},
                       {endTime_, endPose_}};
  poseFunc_ = std::make_shared<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>(waypointPoseList_);
}

void SwingTrajLandingSearch::update(double t)
{
  if(finalizeEndPose_)
  {
    return;
  }

  double preApproachTime = std::next(waypointPoseList_.rbegin(), 2)->first;
  if(t >= preApproachTime)
  {
    finalizeEndPose_ = true;

    // \todo Determine landing pose based on measurements
    sva::PTransformd newEndPose;
    {
      Eigen::Vector3d posOffset = 0.05 * Eigen::Vector3d::Random();
      posOffset.z() = 0.0;
      Eigen::Matrix3d rotOffset = mc_rbdyn::rpyToMat(mc_rtc::constants::toRad(5.0) * Eigen::Vector3d::Random());
      newEndPose = sva::PTransformd(rotOffset, posOffset) * endPose_;
    }

    endPose_ = newEndPose;
    std::next(waypointPoseList_.rbegin())->second = sva::PTransformd(config_.approachOffset) * endPose_;
    waypointPoseList_.rbegin()->second = endPose_;
    poseFunc_ = std::make_shared<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>(waypointPoseList_);
  }
}

sva::PTransformd SwingTrajLandingSearch::pose(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t = touchDownTime_;
  }
  return (*poseFunc_)(t);
}

sva::MotionVecd SwingTrajLandingSearch::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return poseFunc_->derivative(t, 1);
  }
}

sva::MotionVecd SwingTrajLandingSearch::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return poseFunc_->derivative(t, 2);
  }
}

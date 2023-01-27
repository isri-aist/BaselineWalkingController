#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <BaselineWalkingController/swing/SwingTrajVariableTaskGain.h>

using namespace BWC;

void SwingTrajVariableTaskGain::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("verticalTopDurationRatio", verticalTopDurationRatio);
  mcRtcConfig("verticalTopOffset", verticalTopOffset);
}

void SwingTrajVariableTaskGain::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajVariableTaskGain::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
                                               const std::vector<std::string> & category)
{
  gui.addElement(category,
                 mc_rtc::gui::NumberInput(
                     "withdrawDurationRatio", []() { return defaultConfig_.withdrawDurationRatio; },
                     [](double v) { defaultConfig_.withdrawDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "approachDurationRatio", []() { return defaultConfig_.approachDurationRatio; },
                     [](double v) { defaultConfig_.approachDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "verticalTopDurationRatio", []() { return defaultConfig_.verticalTopDurationRatio; },
                     [](double v) { defaultConfig_.verticalTopDurationRatio = v; }),
                 mc_rtc::gui::ArrayInput(
                     "verticalTopOffset", {"x", "y", "z"},
                     []() -> const Eigen::Vector3d & { return defaultConfig_.verticalTopOffset; },
                     [](const Eigen::Vector3d & v) { defaultConfig_.verticalTopOffset = v; }));
}

void SwingTrajVariableTaskGain::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                    const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajVariableTaskGain::SwingTrajVariableTaskGain(const sva::PTransformd & startPose,
                                                     const sva::PTransformd & endPose,
                                                     double startTime,
                                                     double endTime,
                                                     const TaskGain & taskGain,
                                                     const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, endPose, startTime, endTime, taskGain, mcRtcConfig)
{
  config_.load(mcRtcConfig);

  withdrawTime_ = (1 - config_.withdrawDurationRatio) * startTime_ + config_.withdrawDurationRatio * endTime_;
  approachTime_ = config_.approachDurationRatio * startTime_ + (1 - config_.approachDurationRatio) * endTime_;

  // Vertical position
  {
    double verticalTopTime =
        (1.0 - config_.verticalTopDurationRatio) * startTime_ + config_.verticalTopDurationRatio * endTime_;
    TrajColl::BoundaryConstraint<Vector1d> zeroVelBC(TrajColl::BoundaryConstraintType::Velocity, Vector1d::Zero());

    verticalPosFunc_ = std::make_shared<TrajColl::CubicSpline<Vector1d>>(1, zeroVelBC, zeroVelBC);
    verticalPosFunc_->appendPoint(std::make_pair(startTime_, startPose_.translation().tail<1>()));
    verticalPosFunc_->appendPoint(std::make_pair(
        verticalTopTime, (sva::PTransformd(config_.verticalTopOffset) * sva::interpolate(startPose_, endPose_, 0.5))
                             .translation()
                             .tail<1>()));
    verticalPosFunc_->appendPoint(std::make_pair(endTime_, endPose_.translation().tail<1>()));
    verticalPosFunc_->calcCoeff();
  }
}

sva::PTransformd SwingTrajVariableTaskGain::pose(double t) const
{
  sva::PTransformd pose = (t <= withdrawTime_ ? startPose_ : endPose_);
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t = touchDownTime_;
  }
  pose.translation().tail<1>() = (*verticalPosFunc_)(t);
  return pose;
}

sva::MotionVecd SwingTrajVariableTaskGain::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(
        Eigen::Vector3d::Zero(),
        (Eigen::Vector3d() << Eigen::Vector2d::Zero(), verticalPosFunc_->derivative(t, 1)).finished());
  }
}

sva::MotionVecd SwingTrajVariableTaskGain::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(
        Eigen::Vector3d::Zero(),
        (Eigen::Vector3d() << Eigen::Vector2d::Zero(), verticalPosFunc_->derivative(t, 2)).finished());
  }
}

TaskGain SwingTrajVariableTaskGain::taskGain(double t) const
{
  if(t <= withdrawTime_ || approachTime_ <= t)
  {
    return taskGain_;
  }
  else
  {
    double remainingDuration = std::max(approachTime_ - t, 1e-6);
    double stiffness = 6.0 / std::pow(remainingDuration, 2);
    double damping = 4.0 / remainingDuration;
    TaskGain taskGain = TaskGain(sva::MotionVecd(taskGain_.stiffness.vector().cwiseMin(stiffness)),
                                 sva::MotionVecd(taskGain_.damping.vector().cwiseMin(damping)));
    taskGain.stiffness.linear().z() = taskGain_.stiffness.linear().z();
    taskGain.damping.linear().z() = taskGain_.damping.linear().z();
    return taskGain;
  }
}

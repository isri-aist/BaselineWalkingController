#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <BaselineWalkingController/swing/SwingTrajVariableTaskStiffness.h>

using namespace BWC;

void SwingTrajVariableTaskStiffness::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("verticalTopDurationRatio", verticalTopDurationRatio);
  mcRtcConfig("verticalTopOffset", verticalTopOffset);
}

void SwingTrajVariableTaskStiffness::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajVariableTaskStiffness::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
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

void SwingTrajVariableTaskStiffness::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajVariableTaskStiffness::SwingTrajVariableTaskStiffness(const sva::PTransformd & startPose,
                                                               const sva::PTransformd & goalPose,
                                                               double startTime,
                                                               double goalTime,
                                                               const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, goalPose, startTime, goalTime, mcRtcConfig)
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (goalTime - startTime);
  double approachDuration = config_.approachDurationRatio * (goalTime - startTime);

  // Horizontal position
  {
    horizontalPosFunc_ = std::make_shared<CubicInterpolator<Eigen::Vector2d>>();
    horizontalPosFunc_->appendPoint(std::make_pair(startTime, startPose.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(goalTime - approachDuration, goalPose.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(goalTime, goalPose.translation().head<2>()));
    horizontalPosFunc_->calcCoeff();
  }

  // Vertical position
  {
    double verticalTopTime =
        (1.0 - config_.verticalTopDurationRatio) * startTime + config_.verticalTopDurationRatio * goalTime;
    BoundaryConstraint<Vector1d> zeroVelBC(BoundaryConstraintType::Velocity, Vector1d::Zero());

    verticalPosFunc_ = std::make_shared<CubicSpline<Vector1d>>(1, zeroVelBC, zeroVelBC);
    verticalPosFunc_->appendPoint(std::make_pair(startTime, startPose.translation().tail<1>()));
    verticalPosFunc_->appendPoint(std::make_pair(
        verticalTopTime, (sva::PTransformd(config_.verticalTopOffset) * sva::interpolate(startPose, goalPose, 0.5))
                             .translation()
                             .tail<1>()));
    verticalPosFunc_->appendPoint(std::make_pair(goalTime, goalPose.translation().tail<1>()));
    verticalPosFunc_->calcCoeff();
  }

  // Rotation
  {
    rotFunc_ = std::make_shared<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>();
    rotFunc_->appendPoint(std::make_pair(startTime, startPose.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(goalTime - approachDuration, goalPose.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(goalTime, goalPose.rotation().transpose()));
    rotFunc_->calcCoeff();
  }
}

sva::PTransformd SwingTrajVariableTaskStiffness::pose(double t) const
{
  double nominalTime = t;
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    nominalTime = touchDownTime_;
  }
  sva::PTransformd nominalPose = sva::PTransformd(
      (*rotFunc_)(nominalTime).transpose(),
      (Eigen::Vector3d() << (*horizontalPosFunc_)(nominalTime), (*verticalPosFunc_)(nominalTime)).finished());
  return nominalPose;
}

sva::MotionVecd SwingTrajVariableTaskStiffness::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(
        rotFunc_->derivative(t, 1),
        (Eigen::Vector3d() << horizontalPosFunc_->derivative(t, 1), verticalPosFunc_->derivative(t, 1)).finished());
  }
}

sva::MotionVecd SwingTrajVariableTaskStiffness::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(
        rotFunc_->derivative(t, 2),
        (Eigen::Vector3d() << horizontalPosFunc_->derivative(t, 2), verticalPosFunc_->derivative(t, 2)).finished());
  }
}

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <TrajColl/CubicSpline.h>

#include <BaselineWalkingController/swing/SwingTrajCubicSplineSimple.h>

using namespace BWC;

void SwingTrajCubicSplineSimple::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("withdrawOffset", withdrawOffset);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("approachOffset", approachOffset);
  mcRtcConfig("swingOffset", swingOffset);
}

void SwingTrajCubicSplineSimple::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajCubicSplineSimple::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
                                                const std::vector<std::string> & category)
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
          "approachDurationRatio", []() { return defaultConfig_.approachDurationRatio; },
          [](double v) { defaultConfig_.approachDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "approachOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.approachOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.approachOffset = v; }),
      mc_rtc::gui::ArrayInput(
          "swingOffset", {"x", "y", "z"}, []() -> const Eigen::Vector3d & { return defaultConfig_.swingOffset; },
          [](const Eigen::Vector3d & v) { defaultConfig_.swingOffset = v; }));
}

void SwingTrajCubicSplineSimple::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                     const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajCubicSplineSimple::SwingTrajCubicSplineSimple(const sva::PTransformd & startPose,
                                                       const sva::PTransformd & endPose,
                                                       double startTime,
                                                       double endTime,
                                                       const TaskGain & taskGain,
                                                       const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, endPose, startTime, endTime, taskGain, mcRtcConfig),
  posFunc_(std::make_shared<TrajColl::PiecewiseFunc<Eigen::Vector3d>>()),
  rotFunc_(std::make_shared<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>())
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (endTime - startTime);
  double approachDuration = config_.approachDurationRatio * (endTime - startTime);

  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroVelBC(TrajColl::BoundaryConstraintType::Velocity,
                                                          Eigen::Vector3d::Zero());
  TrajColl::BoundaryConstraint<Eigen::Vector3d> zeroAccelBC(TrajColl::BoundaryConstraintType::Acceleration,
                                                            Eigen::Vector3d::Zero());

  // Spline to withdraw foot
  // Pos
  std::map<double, Eigen::Vector3d> withdrawPosWaypoints = {
      {startTime, startPose.translation()},
      {startTime + withdrawDuration, (sva::PTransformd(config_.withdrawOffset) * startPose).translation()}};
  auto withdrawPosSpline =
      std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroVelBC, zeroAccelBC, withdrawPosWaypoints);
  withdrawPosSpline->calcCoeff();
  posFunc_->appendFunc(startTime + withdrawDuration, withdrawPosSpline);
  // Rot
  rotFunc_->appendPoint(std::make_pair(startTime, startPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.rotation().transpose()));

  // Spline to approach foot
  // Pos
  std::map<double, Eigen::Vector3d> approachPosWaypoints = {
      {endTime - approachDuration, (sva::PTransformd(config_.approachOffset) * endPose).translation()},
      {endTime, endPose.translation()}};
  auto approachPosSpline =
      std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(3, zeroAccelBC, zeroVelBC, approachPosWaypoints);
  approachPosSpline->calcCoeff();
  posFunc_->appendFunc(endTime, approachPosSpline);
  // Rot
  rotFunc_->appendPoint(std::make_pair(endTime - approachDuration, endPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(endTime, endPose.rotation().transpose()));

  // Spline to swing foot
  // Pos
  std::map<double, Eigen::Vector3d> swingPosWaypoints = {
      *withdrawPosWaypoints.rbegin(),
      {0.5 * (startTime + endTime),
       (sva::PTransformd(config_.swingOffset) * sva::interpolate(startPose, endPose, 0.5)).translation()},
      *approachPosWaypoints.begin()};
  auto swingPosSpline = std::make_shared<TrajColl::CubicSpline<Eigen::Vector3d>>(
      3,
      TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity,
                                                    withdrawPosSpline->derivative(startTime + withdrawDuration, 1)),
      TrajColl::BoundaryConstraint<Eigen::Vector3d>(TrajColl::BoundaryConstraintType::Velocity,
                                                    approachPosSpline->derivative(endTime - approachDuration, 1)),
      swingPosWaypoints);
  swingPosSpline->calcCoeff();
  posFunc_->appendFunc(endTime - approachDuration, swingPosSpline);
  // Rot
  rotFunc_->calcCoeff();
}

sva::PTransformd SwingTrajCubicSplineSimple::pose(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    t = touchDownTime_;
  }
  return sva::PTransformd((*rotFunc_)(t).transpose(), (*posFunc_)(t));
}

sva::MotionVecd SwingTrajCubicSplineSimple::vel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(rotFunc_->derivative(t, 1), posFunc_->derivative(t, 1));
  }
}

sva::MotionVecd SwingTrajCubicSplineSimple::accel(double t) const
{
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    return sva::MotionVecd::Zero();
  }
  else
  {
    return sva::MotionVecd(rotFunc_->derivative(t, 2), posFunc_->derivative(t, 2));
  }
}

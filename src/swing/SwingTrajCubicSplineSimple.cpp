#include <BaselineWalkingController/swing/SwingTrajCubicSplineSimple.h>
#include <BaselineWalkingController/trajectory/CubicSpline.h>

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

SwingTrajCubicSplineSimple::SwingTrajCubicSplineSimple(const sva::PTransformd & startPose,
                                                       const sva::PTransformd & goalPose,
                                                       double startTime,
                                                       double goalTime,
                                                       const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, goalPose, startTime, goalTime, mcRtcConfig),
  posFunc_(std::make_shared<PiecewiseFunc<Eigen::Vector3d>>()),
  rotFunc_(std::make_shared<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>())
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (goalTime - startTime);
  double approachDuration = config_.approachDurationRatio * (goalTime - startTime);

  BoundaryConstraint<Eigen::Vector3d> zeroVelBC(BoundaryConstraintType::Velocity, Eigen::Vector3d::Zero());
  BoundaryConstraint<Eigen::Vector3d> zeroAccelBC(BoundaryConstraintType::Acceleration, Eigen::Vector3d::Zero());

  // Spline to withdraw foot
  // Pos
  std::map<double, Eigen::Vector3d> withdrawPosWaypoints = {
      {startTime, startPose.translation()},
      {startTime + withdrawDuration, (sva::PTransformd(config_.withdrawOffset) * startPose).translation()}};
  auto withdrawPosSpline =
      std::make_shared<CubicSpline<Eigen::Vector3d>>(3, withdrawPosWaypoints, zeroVelBC, zeroAccelBC);
  withdrawPosSpline->calcCoeff();
  posFunc_->appendFunc(startTime + withdrawDuration, withdrawPosSpline);
  // Rot
  rotFunc_->appendPoint(std::make_pair(startTime, startPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.rotation().transpose()));

  // Spline to approach foot
  // Pos
  std::map<double, Eigen::Vector3d> approachPosWaypoints = {
      {goalTime - approachDuration, (sva::PTransformd(config_.approachOffset) * goalPose).translation()},
      {goalTime, goalPose.translation()}};
  auto approachPosSpline =
      std::make_shared<CubicSpline<Eigen::Vector3d>>(3, approachPosWaypoints, zeroAccelBC, zeroVelBC);
  approachPosSpline->calcCoeff();
  posFunc_->appendFunc(goalTime, approachPosSpline);
  // Rot
  rotFunc_->appendPoint(std::make_pair(goalTime - approachDuration, goalPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(goalTime, goalPose.rotation().transpose()));

  // Spline to swing foot
  // Pos
  std::map<double, Eigen::Vector3d> swingPosWaypoints = {
      *withdrawPosWaypoints.rbegin(),
      {0.5 * (startTime + goalTime),
       (sva::PTransformd(config_.swingOffset) * sva::interpolate(startPose, goalPose, 0.5)).translation()},
      *approachPosWaypoints.begin()};
  auto swingPosSpline = std::make_shared<CubicSpline<Eigen::Vector3d>>(
      3, swingPosWaypoints,
      BoundaryConstraint<Eigen::Vector3d>(BoundaryConstraintType::Velocity,
                                          withdrawPosSpline->derivative(startTime + withdrawDuration, 1)),
      BoundaryConstraint<Eigen::Vector3d>(BoundaryConstraintType::Velocity,
                                          approachPosSpline->derivative(goalTime - approachDuration, 1)));
  swingPosSpline->calcCoeff();
  posFunc_->appendFunc(goalTime - approachDuration, swingPosSpline);
  // Rot
  rotFunc_->calcCoeff();
}

sva::PTransformd SwingTrajCubicSplineSimple::pose(double t) const
{
  return sva::PTransformd((*rotFunc_)(t).transpose(), (*posFunc_)(t));
}

sva::MotionVecd SwingTrajCubicSplineSimple::vel(double t) const
{
  return sva::MotionVecd(rotFunc_->derivative(t, 1), posFunc_->derivative(t, 1));
}

sva::MotionVecd SwingTrajCubicSplineSimple::accel(double t) const
{
  return sva::MotionVecd(rotFunc_->derivative(t, 2), posFunc_->derivative(t, 2));
}

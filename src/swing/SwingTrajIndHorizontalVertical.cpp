#include <BaselineWalkingController/swing/SwingTrajIndHorizontalVertical.h>

using namespace BWC;

void SwingTrajIndHorizontalVertical::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("verticalTopDurationRatio", verticalTopDurationRatio);
  mcRtcConfig("verticalTopOffset", verticalTopOffset);
}

SwingTrajIndHorizontalVertical::SwingTrajIndHorizontalVertical(const sva::PTransformd & startPose,
                                                               const sva::PTransformd & goalPose,
                                                               double startTime,
                                                               double goalTime,
                                                               const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, goalPose, startTime, goalTime, mcRtcConfig)
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (goalTime - startTime);
  double approachDuration = config_.approachDurationRatio * (goalTime - startTime);
  double verticalTopTime =
      (1.0 - config_.verticalTopDurationRatio) * startTime + config_.verticalTopDurationRatio * goalTime;

  horizontalPosFunc_ = std::make_shared<CubicInterpolator<Eigen::Vector2d>>();
  horizontalPosFunc_->appendPoint(std::make_pair(startTime, startPose.translation().head<2>()));
  horizontalPosFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.translation().head<2>()));
  horizontalPosFunc_->appendPoint(std::make_pair(goalTime - approachDuration, goalPose.translation().head<2>()));
  horizontalPosFunc_->appendPoint(std::make_pair(goalTime, goalPose.translation().head<2>()));
  horizontalPosFunc_->calcCoeff();

  BoundaryConstraint<Vector1d> zeroVelBC(BoundaryConstraintType::Velocity, Vector1d::Zero());
  verticalPosFunc_ = std::make_shared<CubicSpline<Vector1d>>(1, std::map<double, Vector1d>{}, zeroVelBC, zeroVelBC);
  verticalPosFunc_->appendPoint(std::make_pair(startTime, startPose.translation().tail<1>()));
  verticalPosFunc_->appendPoint(std::make_pair(
      verticalTopTime, (sva::PTransformd(config_.verticalTopOffset) * sva::interpolate(startPose, goalPose, 0.5))
                           .translation()
                           .tail<1>()));
  verticalPosFunc_->appendPoint(std::make_pair(goalTime, goalPose.translation().tail<1>()));
  verticalPosFunc_->calcCoeff();

  rotFunc_ = std::make_shared<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>();
  rotFunc_->appendPoint(std::make_pair(startTime, startPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(startTime + withdrawDuration, startPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(goalTime - approachDuration, goalPose.rotation().transpose()));
  rotFunc_->appendPoint(std::make_pair(goalTime, goalPose.rotation().transpose()));
  rotFunc_->calcCoeff();
}

sva::PTransformd SwingTrajIndHorizontalVertical::pose(double t) const
{
  Eigen::Vector3d pos;
  pos << (*horizontalPosFunc_)(t), (*verticalPosFunc_)(t);
  return sva::PTransformd((*rotFunc_)(t).transpose(), pos);
}

sva::MotionVecd SwingTrajIndHorizontalVertical::vel(double t) const
{
  Eigen::Vector3d vel;
  vel << horizontalPosFunc_->derivative(t, 1), verticalPosFunc_->derivative(t, 1);
  return sva::MotionVecd(rotFunc_->derivative(t, 1), vel);
}

sva::MotionVecd SwingTrajIndHorizontalVertical::accel(double t) const
{
  Eigen::Vector3d accel;
  accel << horizontalPosFunc_->derivative(t, 2), verticalPosFunc_->derivative(t, 2);
  return sva::MotionVecd(rotFunc_->derivative(t, 2), accel);
}

#include <BaselineWalkingController/swing/SwingTrajIndHorizontalVertical.h>

using namespace BWC;

void SwingTrajIndHorizontalVertical::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("verticalTopDurationRatio", verticalTopDurationRatio);
  mcRtcConfig("verticalTopOffset", verticalTopOffset);
  if(mcRtcConfig.has("withdrawTiltAngle"))
  {
    withdrawTiltAngle = mc_rtc::constants::toRad(mcRtcConfig("withdrawTiltAngle"));
  }
  if(mcRtcConfig.has("approachTiltAngle"))
  {
    approachTiltAngle = mc_rtc::constants::toRad(mcRtcConfig("approachTiltAngle"));
  }
  mcRtcConfig("tiltAngleWithdrawDurationRatio", tiltAngleWithdrawDurationRatio);
  mcRtcConfig("tiltAngleApproachDurationRatio", tiltAngleApproachDurationRatio);
  mcRtcConfig("tiltCenterWithdrawDurationRatio", tiltCenterWithdrawDurationRatio);
  mcRtcConfig("tiltCenterApproachDurationRatio", tiltCenterApproachDurationRatio);
}

SwingTrajIndHorizontalVertical::SwingTrajIndHorizontalVertical(const sva::PTransformd & startPose,
                                                               const sva::PTransformd & goalPose,
                                                               double startTime,
                                                               double goalTime,
                                                               const std::vector<Eigen::Vector3d> & localVertexList,
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
    verticalPosFunc_ = std::make_shared<CubicSpline<Vector1d>>(1, std::map<double, Vector1d>{}, zeroVelBC, zeroVelBC);
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

  // Tilt angle
  {
    double tiltAngleWithdrawDuration = config_.tiltAngleWithdrawDurationRatio * (goalTime - startTime);
    double tiltAngleApproachDuration = config_.tiltAngleApproachDurationRatio * (goalTime - startTime);
    tiltAngleFunc_ = std::make_shared<CubicInterpolator<Vector1d>>();
    tiltAngleFunc_->appendPoint(std::make_pair(startTime, (Vector1d() << 0.0).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(startTime + tiltAngleWithdrawDuration, (Vector1d() << config_.withdrawTiltAngle).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(goalTime - tiltAngleApproachDuration, (Vector1d() << config_.approachTiltAngle).finished()));
    tiltAngleFunc_->appendPoint(std::make_pair(goalTime, (Vector1d() << 0.0).finished()));
    tiltAngleFunc_->calcCoeff();
  }

  // Tilt center
  {
    double tiltCenterWithdrawDuration = config_.tiltCenterWithdrawDurationRatio * (goalTime - startTime);
    double tiltCenterApproachDuration = config_.tiltCenterApproachDurationRatio * (goalTime - startTime);
    Eigen::Vector3d minLocalVertex = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxLocalVertex = Eigen::Vector3d::Zero();
    for(const auto & localVertex : localVertexList)
    {
      minLocalVertex = minLocalVertex.cwiseMin(localVertex);
      maxLocalVertex = maxLocalVertex.cwiseMax(localVertex);
    }
    // \todo
    sva::PTransformd withdrawTiltCenter = sva::PTransformd(Eigen::Vector3d(maxLocalVertex.x(), 0, 0));
    sva::PTransformd approachTiltCenter = sva::PTransformd(Eigen::Vector3d(minLocalVertex.x(), 0, 0));
    tiltCenterFunc_ = std::make_shared<CubicInterpolator<sva::PTransformd, sva::MotionVecd>>();
    tiltCenterFunc_->appendPoint(std::make_pair(startTime, withdrawTiltCenter));
    tiltCenterFunc_->appendPoint(std::make_pair(startTime + withdrawTiltCenterDuration, withdrawTiltCenter));
    tiltCenterFunc_->appendPoint(std::make_pair(goalTime - approachTiltCenterDuration, approachTiltCenter));
    tiltCenterFunc_->appendPoint(std::make_pair(goalTime, approachTiltCenter));
    tiltCenterFunc_->calcCoeff();
  }
}

sva::PTransformd SwingTrajIndHorizontalVertical::pose(double t) const
{
  sva::PTransformd pose = sva::PTransformd(
      (*rotFunc_)(t).transpose(), (Eigen::Vector3d() << (*horizontalPosFunc_)(t), (*verticalPosFunc_)(t)).finished());
  if(tiltAngleFunc_ && tiltCenterFunc_)
  {
    sva::PTransformd tiltCenterTrans = (*tiltCenterFunc_)(t);
    pose = tiltCenterTrans.inv() * sva::PTransformd(sva::RotY((*tiltAngleFunc_)(t)[0])) * tiltCenterTrans * pose;
  }
  return pose;
}

sva::MotionVecd SwingTrajIndHorizontalVertical::vel(double t) const
{
  return sva::MotionVecd(
      rotFunc_->derivative(t, 1),
      (Eigen::Vector3d() << horizontalPosFunc_->derivative(t, 1), verticalPosFunc_->derivative(t, 1)).finished());
}

sva::MotionVecd SwingTrajIndHorizontalVertical::accel(double t) const
{
  return sva::MotionVecd(
      rotFunc_->derivative(t, 2),
      (Eigen::Vector3d() << horizontalPosFunc_->derivative(t, 2), verticalPosFunc_->derivative(t, 2)).finished());
}

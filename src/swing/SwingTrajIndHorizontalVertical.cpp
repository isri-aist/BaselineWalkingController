#include <BaselineWalkingController/swing/SwingTrajIndHorizontalVertical.h>

using namespace BWC;

void SwingTrajIndHorizontalVertical::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  SwingTraj::Configuration::load(mcRtcConfig);

  mcRtcConfig("withdrawDurationRatio", withdrawDurationRatio);
  mcRtcConfig("approachDurationRatio", approachDurationRatio);
  mcRtcConfig("verticalTopDurationRatio", verticalTopDurationRatio);
  mcRtcConfig("verticalTopOffset", verticalTopOffset);
  if(mcRtcConfig.has("tiltAngleWithdraw"))
  {
    tiltAngleWithdraw = mc_rtc::constants::toRad(mcRtcConfig("tiltAngleWithdraw"));
  }
  if(mcRtcConfig.has("tiltAngleApproach"))
  {
    tiltAngleApproach = mc_rtc::constants::toRad(mcRtcConfig("tiltAngleApproach"));
  }
  mcRtcConfig("tiltAngleWithdrawDurationRatio", tiltAngleWithdrawDurationRatio);
  mcRtcConfig("tiltAngleApproachDurationRatio", tiltAngleApproachDurationRatio);
  mcRtcConfig("tiltCenterWithdrawDurationRatio", tiltCenterWithdrawDurationRatio);
  mcRtcConfig("tiltCenterApproachDurationRatio", tiltCenterApproachDurationRatio);
  mcRtcConfig("tiltDistThre", tiltDistThre);
  if(mcRtcConfig.has("tiltForwardAngleThre"))
  {
    tiltForwardAngleThre = mc_rtc::constants::toRad(mcRtcConfig("tiltForwardAngleThre"));
  }
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

  // Determine whether to enable tilt
  int enableTiltWithdraw = 0;
  int enableTiltApproach = 0;
  {
    sva::PTransformd startToGoalTrans = goalPose * startPose.inv();
    sva::PTransformd goalToStartTrans = startPose * goalPose.inv();
    if(startToGoalTrans.translation().norm() > config_.tiltDistThre)
    {
      double forwardAngle =
          std::abs(std::atan2(startToGoalTrans.translation().y(), startToGoalTrans.translation().x()));
      if(forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltWithdraw = 1;
      }
      else if(mc_rtc::constants::PI - forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltWithdraw = -1;
      }
    }
    if(goalToStartTrans.translation().norm() > config_.tiltDistThre)
    {
      double forwardAngle =
          std::abs(std::atan2(goalToStartTrans.translation().y(), goalToStartTrans.translation().x()));
      if(forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltApproach = 1;
      }
      else if(mc_rtc::constants::PI - forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltApproach = -1;
      }
    }
  }

  // Tilt angle
  {
    double tiltAngleWithdrawDuration = config_.tiltAngleWithdrawDurationRatio * (goalTime - startTime);
    double tiltAngleApproachDuration = config_.tiltAngleApproachDurationRatio * (goalTime - startTime);
    double tiltAngleWithdraw = (enableTiltWithdraw == 0 ? 0.0 : enableTiltWithdraw * config_.tiltAngleWithdraw);
    double tiltAngleApproach = (enableTiltApproach == 0 ? 0.0 : enableTiltApproach * config_.tiltAngleApproach);

    tiltAngleFunc_ = std::make_shared<CubicInterpolator<Vector1d>>();
    tiltAngleFunc_->appendPoint(std::make_pair(startTime, (Vector1d() << 0.0).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(startTime + tiltAngleWithdrawDuration, (Vector1d() << tiltAngleWithdraw).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(goalTime - tiltAngleApproachDuration, (Vector1d() << tiltAngleApproach).finished()));
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
    sva::PTransformd tiltCenterWithdraw = sva::PTransformd::Identity();
    if(enableTiltWithdraw == 1)
    {
      tiltCenterWithdraw = sva::PTransformd(Eigen::Vector3d(maxLocalVertex.x(), 0, 0));
    }
    else if(enableTiltWithdraw == -1)
    {
      tiltCenterWithdraw = sva::PTransformd(Eigen::Vector3d(minLocalVertex.x(), 0, 0));
    }
    sva::PTransformd tiltCenterApproach = sva::PTransformd::Identity();
    if(enableTiltApproach == 1)
    {
      tiltCenterApproach = sva::PTransformd(Eigen::Vector3d(maxLocalVertex.x(), 0, 0));
    }
    else if(enableTiltApproach == -1)
    {
      tiltCenterApproach = sva::PTransformd(Eigen::Vector3d(minLocalVertex.x(), 0, 0));
    }

    tiltCenterFunc_ = std::make_shared<CubicInterpolator<sva::PTransformd, sva::MotionVecd>>();
    tiltCenterFunc_->appendPoint(std::make_pair(startTime, tiltCenterWithdraw));
    tiltCenterFunc_->appendPoint(std::make_pair(startTime + tiltCenterWithdrawDuration, tiltCenterWithdraw));
    tiltCenterFunc_->appendPoint(std::make_pair(goalTime - tiltCenterApproachDuration, tiltCenterApproach));
    tiltCenterFunc_->appendPoint(std::make_pair(goalTime, tiltCenterApproach));
    tiltCenterFunc_->calcCoeff();
  }
}

sva::PTransformd SwingTrajIndHorizontalVertical::pose(double t) const
{
  sva::PTransformd nominalPose = sva::PTransformd(
      (*rotFunc_)(t).transpose(), (Eigen::Vector3d() << (*horizontalPosFunc_)(t), (*verticalPosFunc_)(t)).finished());
  sva::PTransformd tiltCenterTrans = (*tiltCenterFunc_)(t);
  return tiltCenterTrans.inv() * sva::PTransformd(sva::RotY((*tiltAngleFunc_)(t)[0])) * tiltCenterTrans * nominalPose;
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

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

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

void SwingTrajIndHorizontalVertical::loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  defaultConfig_.load(mcRtcConfig);
}

void SwingTrajIndHorizontalVertical::addConfigToGUI(mc_rtc::gui::StateBuilder & gui,
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
                     [](const Eigen::Vector3d & v) { defaultConfig_.verticalTopOffset = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltAngleWithdraw", []() { return mc_rtc::constants::toDeg(defaultConfig_.tiltAngleWithdraw); },
                     [](double v) { defaultConfig_.tiltAngleWithdraw = mc_rtc::constants::toRad(v); }),
                 mc_rtc::gui::NumberInput(
                     "tiltAngleApproach", []() { return mc_rtc::constants::toDeg(defaultConfig_.tiltAngleApproach); },
                     [](double v) { defaultConfig_.tiltAngleApproach = mc_rtc::constants::toRad(v); }),
                 mc_rtc::gui::NumberInput(
                     "tiltAngleWithdrawDurationRatio", []() { return defaultConfig_.tiltAngleWithdrawDurationRatio; },
                     [](double v) { defaultConfig_.tiltAngleWithdrawDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltAngleApproachDurationRatio", []() { return defaultConfig_.tiltAngleApproachDurationRatio; },
                     [](double v) { defaultConfig_.tiltAngleApproachDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltCenterWithdrawDurationRatio", []() { return defaultConfig_.tiltCenterWithdrawDurationRatio; },
                     [](double v) { defaultConfig_.tiltCenterWithdrawDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltCenterApproachDurationRatio", []() { return defaultConfig_.tiltCenterApproachDurationRatio; },
                     [](double v) { defaultConfig_.tiltCenterApproachDurationRatio = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltDistThre", []() { return defaultConfig_.tiltDistThre; },
                     [](double v) { defaultConfig_.tiltDistThre = v; }),
                 mc_rtc::gui::NumberInput(
                     "tiltForwardAngleThre",
                     []() { return mc_rtc::constants::toDeg(defaultConfig_.tiltForwardAngleThre); },
                     [](double v) { defaultConfig_.tiltForwardAngleThre = mc_rtc::constants::toRad(v); }));
}

void SwingTrajIndHorizontalVertical::removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

SwingTrajIndHorizontalVertical::SwingTrajIndHorizontalVertical(const sva::PTransformd & startPose,
                                                               const sva::PTransformd & endPose,
                                                               double startTime,
                                                               double endTime,
                                                               const TaskGain & taskGain,
                                                               const mc_rtc::Configuration & mcRtcConfig)
: SwingTraj(startPose, endPose, startTime, endTime, taskGain, mcRtcConfig)
{
  config_.load(mcRtcConfig);

  double withdrawDuration = config_.withdrawDurationRatio * (endTime_ - startTime_);
  double approachDuration = config_.approachDurationRatio * (endTime_ - startTime_);

  // Horizontal position
  {
    horizontalPosFunc_ = std::make_shared<TrajColl::CubicInterpolator<Eigen::Vector2d>>();
    horizontalPosFunc_->appendPoint(std::make_pair(startTime_, startPose_.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(startTime_ + withdrawDuration, startPose_.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(endTime_ - approachDuration, endPose_.translation().head<2>()));
    horizontalPosFunc_->appendPoint(std::make_pair(endTime_, endPose_.translation().head<2>()));
    horizontalPosFunc_->calcCoeff();
  }

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

  // Rotation
  {
    rotFunc_ = std::make_shared<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>();
    rotFunc_->appendPoint(std::make_pair(startTime_, startPose_.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(startTime_ + withdrawDuration, startPose_.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(endTime_ - approachDuration, endPose_.rotation().transpose()));
    rotFunc_->appendPoint(std::make_pair(endTime_, endPose_.rotation().transpose()));
    rotFunc_->calcCoeff();
  }

  // Determine whether to enable tilt
  int enableTiltWithdraw = 0;
  int enableTiltApproach = 0;
  {
    sva::PTransformd startToEndTrans = endPose_ * startPose_.inv();
    sva::PTransformd endToStartTrans = startPose_ * endPose_.inv();
    if(startToEndTrans.translation().norm() > config_.tiltDistThre)
    {
      double forwardAngle = std::abs(std::atan2(startToEndTrans.translation().y(), startToEndTrans.translation().x()));
      if(forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltWithdraw = 1;
      }
      else if(mc_rtc::constants::PI - forwardAngle < config_.tiltForwardAngleThre)
      {
        enableTiltWithdraw = -1;
      }
    }
    if(endToStartTrans.translation().norm() > config_.tiltDistThre)
    {
      double forwardAngle = std::abs(std::atan2(endToStartTrans.translation().y(), endToStartTrans.translation().x()));
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
    double tiltAngleWithdrawDuration = config_.tiltAngleWithdrawDurationRatio * (endTime_ - startTime_);
    double tiltAngleApproachDuration = config_.tiltAngleApproachDurationRatio * (endTime_ - startTime_);
    double tiltAngleWithdraw = (enableTiltWithdraw == 0 ? 0.0 : enableTiltWithdraw * config_.tiltAngleWithdraw);
    double tiltAngleApproach = (enableTiltApproach == 0 ? 0.0 : enableTiltApproach * config_.tiltAngleApproach);

    tiltAngleFunc_ = std::make_shared<TrajColl::CubicInterpolator<Vector1d>>();
    tiltAngleFunc_->appendPoint(std::make_pair(startTime_, (Vector1d() << 0.0).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(startTime_ + tiltAngleWithdrawDuration, (Vector1d() << tiltAngleWithdraw).finished()));
    tiltAngleFunc_->appendPoint(
        std::make_pair(endTime_ - tiltAngleApproachDuration, (Vector1d() << tiltAngleApproach).finished()));
    tiltAngleFunc_->appendPoint(std::make_pair(endTime_, (Vector1d() << 0.0).finished()));
    tiltAngleFunc_->calcCoeff();
  }

  // Tilt center
  {
    double tiltCenterWithdrawDuration = config_.tiltCenterWithdrawDurationRatio * (endTime_ - startTime_);
    double tiltCenterApproachDuration = config_.tiltCenterApproachDurationRatio * (endTime_ - startTime_);

    Eigen::Vector3d minLocalVertex = Eigen::Vector3d::Zero();
    Eigen::Vector3d maxLocalVertex = Eigen::Vector3d::Zero();
    if(mcRtcConfig.has("localVertexList"))
    {
      for(const auto & localVertex : static_cast<std::vector<Eigen::Vector3d>>(mcRtcConfig("localVertexList")))
      {
        minLocalVertex = minLocalVertex.cwiseMin(localVertex);
        maxLocalVertex = maxLocalVertex.cwiseMax(localVertex);
      }
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

    tiltCenterFunc_ = std::make_shared<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>();
    tiltCenterFunc_->appendPoint(std::make_pair(startTime_, tiltCenterWithdraw));
    tiltCenterFunc_->appendPoint(std::make_pair(startTime_ + tiltCenterWithdrawDuration, tiltCenterWithdraw));
    tiltCenterFunc_->appendPoint(std::make_pair(endTime_ - tiltCenterApproachDuration, tiltCenterApproach));
    tiltCenterFunc_->appendPoint(std::make_pair(endTime_, tiltCenterApproach));
    tiltCenterFunc_->calcCoeff();
  }
}

sva::PTransformd SwingTrajIndHorizontalVertical::pose(double t) const
{
  double nominalTime = t;
  if(touchDownTime_ > 0 && t >= touchDownTime_)
  {
    nominalTime = touchDownTime_;
  }
  sva::PTransformd nominalPose = sva::PTransformd(
      (*rotFunc_)(nominalTime).transpose(),
      (Eigen::Vector3d() << (*horizontalPosFunc_)(nominalTime), (*verticalPosFunc_)(nominalTime)).finished());
  sva::PTransformd tiltCenterTrans = (*tiltCenterFunc_)(t);
  return tiltCenterTrans.inv() * sva::PTransformd(sva::RotY((*tiltAngleFunc_)(t)[0])) * tiltCenterTrans * nominalPose;
}

sva::MotionVecd SwingTrajIndHorizontalVertical::vel(double t) const
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

sva::MotionVecd SwingTrajIndHorizontalVertical::accel(double t) const
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

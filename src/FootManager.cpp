#include <limits>

#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/tasks/FirstOrderImpedanceTask.h>
#include <BaselineWalkingController/trajectory/CubicSpline.h>
#include <BaselineWalkingController/wrench/Contact.h>

using namespace BWC;

Foot BWC::strToFoot(const std::string & footStr)
{
  if(footStr == "Left")
  {
    return Foot::Left;
  }
  else if(footStr == "Right")
  {
    return Foot::Right;
  }
  else
  {
    mc_rtc::log::error_and_throw("[strToFoot] Unsupported Foot name: {}", footStr);
  }
}

Foot BWC::opposite(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return Foot::Right;
  }
  else // if(footStr == "Right")
  {
    return Foot::Left;
  }
}

int BWC::sign(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return 1;
  }
  else // if(footStr == "Right")
  {
    return -1;
  }
}

void FootManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("footstepDuration", footstepDuration);
  mcRtcConfig("doubleSupportRatio", doubleSupportRatio);
  if(mcRtcConfig.has("midToFootTranss"))
  {
    for(const auto & foot : Feet::Both)
    {
      mcRtcConfig("midToFootTranss")(std::to_string(foot), midToFootTranss.at(foot));
    }
  }
  mcRtcConfig("swingHeight", swingHeight);
  mcRtcConfig("swingInitialLiftHeight", swingInitialLiftHeight);
  mcRtcConfig("swingInitialLiftDurationRatio", swingInitialLiftDurationRatio);
  mcRtcConfig("zmpHorizon", zmpHorizon);
  mcRtcConfig("zmpOffset", zmpOffset);
  mcRtcConfig("stopSwingTrajForTouchDownFoot", stopSwingTrajForTouchDownFoot);
  mcRtcConfig("keepSupportFootPoseForTouchDownFoot", keepSupportFootPoseForTouchDownFoot);
  mcRtcConfig("enableWrenchDistForTouchDownFoot", enableWrenchDistForTouchDownFoot);
  mcRtcConfig("fricCoeff", fricCoeff);
  mcRtcConfig("touchDownRemainingDuration", touchDownRemainingDuration);
  mcRtcConfig("touchDownPosError", touchDownPosError);
  mcRtcConfig("touchDownForceZ", touchDownForceZ);
  if(mcRtcConfig.has("impedanceGains"))
  {
    mcRtcConfig("impedanceGains")("singleSupport", impGains.at("singleSupport"));
    mcRtcConfig("impedanceGains")("doubleSupport", impGains.at("doubleSupport"));
    mcRtcConfig("impedanceGains")("swing", impGains.at("swing"));
  }
}

FootManager::FootManager(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr), zmpTrajFunc_(std::make_shared<CubicInterpolator<Eigen::Vector3d>>()),
  swingPosFunc_(std::make_shared<PiecewiseFunc<Eigen::Vector3d>>()),
  swingRotFunc_(std::make_shared<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>>())
{
  config_.load(mcRtcConfig);
}

void FootManager::reset()
{
  footstepQueue_.clear();

  for(const auto & foot : Feet::Both)
  {
    targetFootPoses_.emplace(foot, ctl().robot().surfacePose(surfaceName(foot)));
    targetFootVels_.emplace(foot, sva::MotionVecd::Zero());
    targetFootAccels_.emplace(foot, sva::MotionVecd::Zero());
  }

  supportPhase_ = SupportPhase::DoubleSupport;

  Eigen::Vector3d targetZmp = calcZmpWithOffset(targetFootPoses_);
  zmpTrajFunc_->clearPoints();
  zmpTrajFunc_->appendPoint(std::make_pair(ctl().t(), targetZmp));
  zmpTrajFunc_->appendPoint(std::make_pair(ctl().t() + config_.zmpHorizon, targetZmp));
  zmpTrajFunc_->calcCoeff();

  swingFootstep_ = nullptr;

  swingPosFunc_->clearFuncs();
  swingRotFunc_->clearPoints();

  touchDown_ = false;

  for(const auto & foot : Feet::Both)
  {
    impGainTypes_.emplace(foot, "doubleSupport");
  }

  requireImpGainUpdate_ = true;
}

void FootManager::update()
{
  updateFootTasks();
  updateZmpTraj();
}

void FootManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"BWC", config_.name, "Status"},
                 mc_rtc::gui::Label("supportPhase", [this]() { return std::to_string(supportPhase_); }),
                 mc_rtc::gui::Label("footstepQueueSize", [this]() { return std::to_string(footstepQueue_.size()); }));
  gui.addElement({"BWC", config_.name, "Status"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("LeftFootSurface", [this]() { return surfaceName(Foot::Left); }),
                 mc_rtc::gui::Label("RightFootSurface", [this]() { return surfaceName(Foot::Right); }));
  gui.addElement({"BWC", config_.name, "Status"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("LeftImpGainType", [this]() { return impGainTypes_.at(Foot::Left); }),
                 mc_rtc::gui::Label("RightImpGainType", [this]() { return impGainTypes_.at(Foot::Right); }));

  gui.addElement(
      {"BWC", config_.name, "Configuration"},
      mc_rtc::gui::NumberInput(
          "footstepDuration", [this]() { return config_.footstepDuration; },
          [this](double v) { config_.footstepDuration = v; }),
      mc_rtc::gui::NumberInput(
          "doubleSupportRatio", [this]() { return config_.doubleSupportRatio; },
          [this](double v) { config_.doubleSupportRatio = v; }),
      mc_rtc::gui::NumberInput(
          "swingHeight", [this]() { return config_.swingHeight; }, [this](double v) { config_.swingHeight = v; }),
      mc_rtc::gui::NumberInput(
          "swingInitialLiftHeight", [this]() { return config_.swingInitialLiftHeight; },
          [this](double v) { config_.swingInitialLiftHeight = v; }),
      mc_rtc::gui::NumberInput(
          "swingInitialLiftDurationRatio", [this]() { return config_.swingInitialLiftDurationRatio; },
          [this](double v) { config_.swingInitialLiftDurationRatio = v; }),
      mc_rtc::gui::ArrayInput(
          "zmpOffset", {"x", "y", "z"}, [this]() -> const Eigen::Vector3d { return config_.zmpOffset; },
          [this](const Eigen::Vector3d & v) { config_.zmpOffset = v; }),
      mc_rtc::gui::Checkbox(
          "stopSwingTrajForTouchDownFoot", [this]() { return config_.stopSwingTrajForTouchDownFoot; },
          [this]() { config_.stopSwingTrajForTouchDownFoot = !config_.stopSwingTrajForTouchDownFoot; }),
      mc_rtc::gui::Checkbox(
          "keepSupportFootPoseForTouchDownFoot", [this]() { return config_.keepSupportFootPoseForTouchDownFoot; },
          [this]() { config_.keepSupportFootPoseForTouchDownFoot = !config_.keepSupportFootPoseForTouchDownFoot; }),
      mc_rtc::gui::Checkbox(
          "enableWrenchDistForTouchDownFoot", [this]() { return config_.enableWrenchDistForTouchDownFoot; },
          [this]() { config_.enableWrenchDistForTouchDownFoot = !config_.enableWrenchDistForTouchDownFoot; }),
      mc_rtc::gui::NumberInput(
          "fricCoeff", [this]() { return config_.fricCoeff; }, [this](double v) { config_.fricCoeff = v; }),
      mc_rtc::gui::NumberInput(
          "touchDownRemainingDuration", [this]() { return config_.touchDownRemainingDuration; },
          [this](double v) { config_.touchDownRemainingDuration = v; }),
      mc_rtc::gui::NumberInput(
          "touchDownPosError", [this]() { return config_.touchDownPosError; },
          [this](double v) { config_.touchDownPosError = v; }),
      mc_rtc::gui::NumberInput(
          "touchDownForceZ", [this]() { return config_.touchDownForceZ; },
          [this](double v) { config_.touchDownForceZ = v; }));

  for(const auto & impGainKV : config_.impGains)
  {
    const auto & impGainType = impGainKV.first;
    gui.addElement({"BWC", config_.name, "ImpedanceGains", impGainType},
                   mc_rtc::gui::ArrayInput(
                       "Damper", {"cx", "cy", "cz", "fx", "fy", "fz"},
                       [this, impGainType]() -> const sva::ImpedanceVecd & {
                         return config_.impGains.at(impGainType).damper().vec();
                       },
                       [this, impGainType](const Eigen::Vector6d & v) {
                         config_.impGains.at(impGainType).damper().vec(v);
                         requireImpGainUpdate_ = true;
                       }));
    gui.addElement({"BWC", config_.name, "ImpedanceGains", impGainType},
                   mc_rtc::gui::ArrayInput(
                       "Spring", {"cx", "cy", "cz", "fx", "fy", "fz"},
                       [this, impGainType]() -> const sva::ImpedanceVecd & {
                         return config_.impGains.at(impGainType).spring().vec();
                       },
                       [this, impGainType](const Eigen::Vector6d & v) {
                         config_.impGains.at(impGainType).spring().vec(v);
                         requireImpGainUpdate_ = true;
                       }));
    gui.addElement({"BWC", config_.name, "ImpedanceGains", impGainType},
                   mc_rtc::gui::ArrayInput(
                       "Wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                       [this, impGainType]() -> const sva::ImpedanceVecd & {
                         return config_.impGains.at(impGainType).wrench().vec();
                       },
                       [this, impGainType](const Eigen::Vector6d & v) {
                         config_.impGains.at(impGainType).wrench().vec(v);
                         requireImpGainUpdate_ = true;
                       }));
  }
}

void FootManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"BWC", config_.name});
}

void FootManager::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(config_.name + "_footstepQueueSize", this, [this]() { return footstepQueue_.size(); });

  for(const auto & foot : Feet::Both)
  {
    logger.addLogEntry(config_.name + "_targetFootPose_" + std::to_string(foot), this,
                       [this, foot]() -> const sva::PTransformd & { return targetFootPoses_.at(foot); });

    logger.addLogEntry(config_.name + "_targetFootVel_" + std::to_string(foot), this,
                       [this, foot]() -> const sva::MotionVecd & { return targetFootVels_.at(foot); });

    logger.addLogEntry(config_.name + "_targetFootAccel_" + std::to_string(foot), this,
                       [this, foot]() -> const sva::MotionVecd & { return targetFootAccels_.at(foot); });
  }

  logger.addLogEntry(config_.name + "_supportPhase", this, [this]() { return std::to_string(supportPhase_); });

  logger.addLogEntry(config_.name + "_refZmp", this, [this]() { return (*zmpTrajFunc_)(ctl().t()); });

  logger.addLogEntry(config_.name + "_leftFootSupportRatio", this, [this]() { return leftFootSupportRatio(); });

  logger.addLogEntry(config_.name + "_touchDown", this, [this]() { return touchDown_; });

  logger.addLogEntry(config_.name + "_touchDownRemainingDuration", this,
                     [this]() { return touchDownRemainingDuration(); });

  for(const auto & foot : Feet::Both)
  {
    logger.addLogEntry(config_.name + "_impGainType_" + std::to_string(foot), this,
                       [this, foot]() -> const std::string & { return impGainTypes_.at(foot); });
  }
}

void FootManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

const std::string & FootManager::surfaceName(const Foot & foot) const
{
  return ctl().footTasks_.at(foot)->surface();
}

Footstep FootManager::makeFootstep(const Foot & foot, const sva::PTransformd & footMidpose, double startTime) const
{
  return Footstep(foot, config_.midToFootTranss.at(foot) * footMidpose, startTime,
                  startTime + 0.5 * config_.doubleSupportRatio * config_.footstepDuration,
                  startTime + (1.0 - 0.5 * config_.doubleSupportRatio) * config_.footstepDuration,
                  startTime + config_.footstepDuration);
}

bool FootManager::appendFootstep(const Footstep & newFootstep)
{
  // Check time of new footstep
  if(newFootstep.transitStartTime < ctl().t())
  {
    mc_rtc::log::error("[FootManager] Ignore a new footstep with past time: {} < {}", newFootstep.transitStartTime,
                       ctl().t());
    return false;
  }
  if(!footstepQueue_.empty())
  {
    const Footstep & lastFootstep = footstepQueue_.back();
    if(newFootstep.transitStartTime < lastFootstep.transitEndTime)
    {
      mc_rtc::log::error("[FootManager] Ignore a new footstep earlier than the last footstep: {} < {}",
                         newFootstep.transitStartTime, lastFootstep.transitEndTime);
      return false;
    }
  }

  // Push to the queue
  footstepQueue_.push_back(newFootstep);

  return true;
}

Eigen::Vector3d FootManager::calcRefZmp(double t, int derivOrder) const
{
  if(derivOrder == 0)
  {
    return (*zmpTrajFunc_)(t);
  }
  else
  {
    return zmpTrajFunc_->derivative(t, derivOrder);
  }
}

std::set<Foot> FootManager::getContactFeet() const
{
  if(supportPhase_ == SupportPhase::DoubleSupport)
  {
    return Feet::Both;
  }
  else
  {
    if(config_.enableWrenchDistForTouchDownFoot && touchDown_)
    {
      return Feet::Both;
    }
    else
    {
      if(supportPhase_ == SupportPhase::LeftSupport)
      {
        return std::set<Foot>{Foot::Left};
      }
      else // if(supportPhase_ == SupportPhase::RightSupport)
      {
        return std::set<Foot>{Foot::Right};
      }
    }
  }
}

std::unordered_map<Foot, std::shared_ptr<Contact>> FootManager::calcContactList() const
{
  // Set contactList
  std::unordered_map<Foot, std::shared_ptr<Contact>> contactList;
  for(const auto & foot : getContactFeet())
  {
    std::vector<Eigen::Vector3d> localVertexList;
    const auto & surface = ctl().robot().surface(surfaceName(foot));
    for(const auto & point : surface.points())
    {
      // Surface points are represented in body frame, not surface frame
      localVertexList.push_back((point * surface.X_b_s().inv()).translation());
    }
    contactList.emplace(foot, std::make_shared<Contact>(std::to_string(foot), config_.fricCoeff, localVertexList,
                                                        targetFootPoses_.at(foot)));
  }

  return contactList;
}

double FootManager::leftFootSupportRatio() const
{
  if(footstepQueue_.empty())
  {
    return 0.5;
  }

  const Footstep & footstep = footstepQueue_.front();

  if(ctl().t() <= footstep.transitStartTime)
  {
    return 0.5;
  }
  else if(ctl().t() <= footstep.swingStartTime)
  {
    double ratio = (ctl().t() - footstep.transitStartTime) / (footstep.swingStartTime - footstep.transitStartTime);
    return -0.5 * sign(footstep.foot) * mc_filter::utils::clamp(ratio, 0.0, 1.0) + 0.5;
  }
  else if(ctl().t() <= footstep.swingEndTime)
  {
    return footstep.foot == Foot::Left ? 0 : 1;
  }
  else // if(ctl().t() <= footstep.transitEndTime)
  {
    double ratio = (footstep.transitEndTime - ctl().t()) / (footstep.transitEndTime - footstep.swingEndTime);
    return -0.5 * sign(footstep.foot) * mc_filter::utils::clamp(ratio, 0.0, 1.0) + 0.5;
  }
}

void FootManager::updateFootTasks()
{
  // Remove old footsteps from footstepQueue_
  while(!footstepQueue_.empty() && footstepQueue_.front().transitEndTime < ctl().t())
  {
    footstepQueue_.pop_front();
  }

  if(!footstepQueue_.empty() && footstepQueue_.front().swingStartTime <= ctl().t()
     && ctl().t() <= footstepQueue_.front().swingEndTime)
  {
    // Single support phase
    if(swingFootstep_)
    {
      // Check if swingFootstep_ is consistent
      if(swingFootstep_ != &(footstepQueue_.front()))
      {
        mc_rtc::log::error_and_throw("[FootManager] Swing footstep is not consistent.");
      }
    }
    else
    {
      // Set swingFootstep_
      swingFootstep_ = &(footstepQueue_.front());

      // Set swingPosFunc_ and swingRotFunc_
      {
        const sva::PTransformd & swingStartPose = ctl().robot().surfacePose(surfaceName(swingFootstep_->foot));
        const sva::PTransformd & swingGoalPose = swingFootstep_->pose;
        sva::PTransformd swingMidPose = sva::interpolate(swingStartPose, swingGoalPose, 0.5);
        double swingInitialLiftDuration =
            config_.swingInitialLiftDurationRatio * (swingFootstep_->swingEndTime - swingFootstep_->swingStartTime);

        BoundaryConstraint<Eigen::Vector3d> zeroVelBC(BoundaryConstraintType::Velocity, Eigen::Vector3d::Zero());
        BoundaryConstraint<Eigen::Vector3d> zeroAccelBC(BoundaryConstraintType::Acceleration, Eigen::Vector3d::Zero());

        // Spline for initial lift up
        // Pos
        std::map<double, Eigen::Vector3d> liftUpWayPosPoints = {
            {swingFootstep_->swingStartTime, swingStartPose.translation()},
            {swingFootstep_->swingStartTime + swingInitialLiftDuration,
             (sva::PTransformd(Eigen::Vector3d(0, 0, config_.swingInitialLiftHeight)) * swingStartPose).translation()}};
        auto liftUpPosSpline =
            std::make_shared<CubicSpline<Eigen::Vector3d>>(3, liftUpWayPosPoints, zeroVelBC, zeroAccelBC);
        liftUpPosSpline->calcCoeff();
        swingPosFunc_->appendFunc(swingFootstep_->swingStartTime + swingInitialLiftDuration, liftUpPosSpline);
        // Rot
        swingRotFunc_->appendPoint(std::make_pair(swingFootstep_->swingStartTime, swingStartPose.rotation()));
        swingRotFunc_->appendPoint(
            std::make_pair(swingFootstep_->swingStartTime + swingInitialLiftDuration, swingStartPose.rotation()));

        // Spline for final lift down
        // Pos
        std::map<double, Eigen::Vector3d> liftDownWayPosPoints = {
            {swingFootstep_->swingEndTime - swingInitialLiftDuration,
             (sva::PTransformd(Eigen::Vector3d(0, 0, config_.swingInitialLiftHeight)) * swingGoalPose).translation()},
            {swingFootstep_->swingEndTime, swingGoalPose.translation()}};
        auto liftDownPosSpline =
            std::make_shared<CubicSpline<Eigen::Vector3d>>(3, liftDownWayPosPoints, zeroAccelBC, zeroVelBC);
        liftDownPosSpline->calcCoeff();
        swingPosFunc_->appendFunc(swingFootstep_->swingEndTime, liftDownPosSpline);
        // Rot
        swingRotFunc_->appendPoint(
            std::make_pair(swingFootstep_->swingEndTime - swingInitialLiftDuration, swingFootstep_->pose.rotation()));
        swingRotFunc_->appendPoint(std::make_pair(swingFootstep_->swingEndTime, swingFootstep_->pose.rotation()));

        // Spline for swing
        // Pos
        std::map<double, Eigen::Vector3d> swingWayPosPoints = {
            *liftUpWayPosPoints.rbegin(),
            {0.5 * (swingFootstep_->swingStartTime + swingFootstep_->swingEndTime),
             Eigen::Vector3d(swingMidPose.translation().x(), swingMidPose.translation().y(),
                             std::max(swingStartPose.translation().z(), swingGoalPose.translation().z())
                                 + config_.swingHeight)},
            *liftDownWayPosPoints.begin()};
        auto swingPosSpline = std::make_shared<CubicSpline<Eigen::Vector3d>>(
            3, swingWayPosPoints,
            BoundaryConstraint<Eigen::Vector3d>(
                BoundaryConstraintType::Velocity,
                liftUpPosSpline->derivative(swingFootstep_->swingStartTime + swingInitialLiftDuration, 1)),
            BoundaryConstraint<Eigen::Vector3d>(
                BoundaryConstraintType::Velocity,
                liftDownPosSpline->derivative(swingFootstep_->swingEndTime - swingInitialLiftDuration, 1)));
        swingPosSpline->calcCoeff();
        swingPosFunc_->appendFunc(swingFootstep_->swingEndTime - swingInitialLiftDuration, swingPosSpline);
        // Rot
        swingRotFunc_->calcCoeff();
      }

      // Set supportPhase_
      if(swingFootstep_->foot == Foot::Left)
      {
        supportPhase_ = SupportPhase::RightSupport;
      }
      else
      {
        supportPhase_ = SupportPhase::LeftSupport;
      }
    }

    // Update target
    if(!(config_.stopSwingTrajForTouchDownFoot && touchDown_))
    {
      targetFootPoses_.at(swingFootstep_->foot) =
          sva::PTransformd((*swingRotFunc_)(ctl().t()), (*swingPosFunc_)(ctl().t()));
      targetFootVels_.at(swingFootstep_->foot) =
          sva::MotionVecd(swingRotFunc_->derivative(ctl().t(), 1), swingPosFunc_->derivative(ctl().t(), 1));
      targetFootAccels_.at(swingFootstep_->foot) =
          sva::MotionVecd(swingRotFunc_->derivative(ctl().t(), 2), swingPosFunc_->derivative(ctl().t(), 2));
    }

    // Update touchDown_
    if(!touchDown_ && detectTouchDown())
    {
      touchDown_ = true;

      if(config_.stopSwingTrajForTouchDownFoot)
      {
        targetFootVels_.at(swingFootstep_->foot) = sva::MotionVecd::Zero();
        targetFootAccels_.at(swingFootstep_->foot) = sva::MotionVecd::Zero();
      }
    }
  }
  else
  {
    // Double support phase
    if(swingFootstep_)
    {
      // Update target
      if(!(config_.keepSupportFootPoseForTouchDownFoot && touchDown_))
      {
        targetFootPoses_.at(swingFootstep_->foot) = sva::PTransformd((*swingRotFunc_)(swingFootstep_->swingEndTime),
                                                                     (*swingPosFunc_)(swingFootstep_->swingEndTime));
        targetFootVels_.at(swingFootstep_->foot) = sva::MotionVecd::Zero();
        targetFootAccels_.at(swingFootstep_->foot) = sva::MotionVecd::Zero();
      }

      // Set supportPhase_
      supportPhase_ = SupportPhase::DoubleSupport;

      // Clear swingPosFunc_ and swingRotFunc_
      swingPosFunc_->clearFuncs();
      swingRotFunc_->clearPoints();

      // Clear touchDown_
      touchDown_ = false;

      // Clear swingFootstep_
      swingFootstep_ = nullptr;
    }
  }

  // Update task target
  for(const auto & foot : Feet::Both)
  {
    ctl().footTasks_.at(foot)->targetPose(targetFootPoses_.at(foot));
    // ImpedanceTask::targetVel receive the velocity represented in the world frame
    ctl().footTasks_.at(foot)->targetVel(targetFootVels_.at(foot));
    // ImpedanceTask::targetAccel receive the acceleration represented in the world frame
    ctl().footTasks_.at(foot)->targetAccel(targetFootAccels_.at(foot));
  }

  // Update impGainTypes_ and requireImpGainUpdate_
  std::unordered_map<Foot, std::string> newImpGainTypes;
  const auto & contactFeet = getContactFeet();
  if(contactFeet.size() == 1)
  {
    newImpGainTypes.emplace(*(contactFeet.cbegin()), "singleSupport");
    newImpGainTypes.emplace(opposite(*(contactFeet.cbegin())), "swing");
  }
  else // if(contactFeet.size() == 2)
  {
    for(const auto & foot : Feet::Both)
    {
      newImpGainTypes.emplace(foot, "doubleSupport");
    }
  }
  for(const auto & foot : Feet::Both)
  {
    if(requireImpGainUpdate_)
    {
      break;
    }
    if(impGainTypes_.at(foot) != newImpGainTypes.at(foot))
    {
      requireImpGainUpdate_ = true;
    }
  }
  impGainTypes_ = newImpGainTypes;

  // Update task impedance gains
  if(requireImpGainUpdate_)
  {
    requireImpGainUpdate_ = false;

    for(const auto & foot : Feet::Both)
    {
      ctl().footTasks_.at(foot)->gains() = config_.impGains.at(impGainTypes_.at(foot));
    }
  }

  // Update footstep visualization
  std::vector<std::vector<Eigen::Vector3d>> footstepPolygonList;
  for(const auto & footstep : footstepQueue_)
  {
    std::vector<Eigen::Vector3d> footstepPolygon;
    const auto & surface = ctl().robot().surface(surfaceName(footstep.foot));
    for(const auto & point : surface.points())
    {
      // Surface points are represented in body frame, not surface frame
      footstepPolygon.push_back((point * surface.X_b_s().inv() * footstep.pose).translation());
    }
    footstepPolygonList.push_back(footstepPolygon);
  }

  ctl().gui()->removeCategory({"BWC", config_.name, "FootstepMarker"});
  ctl().gui()->addElement({"BWC", config_.name, "FootstepMarker"},
                          mc_rtc::gui::Polygon("Footstep", {mc_rtc::gui::Color::Blue, 0.02},
                                               [footstepPolygonList]() { return footstepPolygonList; }));
}

void FootManager::updateZmpTraj()
{
  zmpTrajFunc_->clearPoints();

  std::unordered_map<Foot, sva::PTransformd> footPoses = targetFootPoses_;

  if(footstepQueue_.empty() || ctl().t() < footstepQueue_.front().transitStartTime)
  {
    // Set ZMP at the foot middle position
    zmpTrajFunc_->appendPoint(std::make_pair(ctl().t(), calcZmpWithOffset(footPoses)));
  }

  for(const auto & focusedFootstep : footstepQueue_)
  {
    Foot supportFoot = opposite(focusedFootstep.foot);
    Eigen::Vector3d supportFootZmp = calcZmpWithOffset(supportFoot, footPoses.at(supportFoot));

    // Keep ZMP at the foot middle position
    if(ctl().t() < focusedFootstep.swingStartTime)
    {
      zmpTrajFunc_->appendPoint(std::make_pair(focusedFootstep.transitStartTime, calcZmpWithOffset(footPoses)));
    }

    // Move ZMP to the support foot position
    zmpTrajFunc_->appendPoint(std::make_pair(focusedFootstep.swingStartTime, supportFootZmp));

    // Keep ZMP at the support foot position
    zmpTrajFunc_->appendPoint(std::make_pair(focusedFootstep.swingEndTime, supportFootZmp));

    // Update footPoses
    footPoses.at(focusedFootstep.foot) = focusedFootstep.pose;

    // Move ZMP to the foot middle position
    zmpTrajFunc_->appendPoint(std::make_pair(focusedFootstep.transitEndTime, calcZmpWithOffset(footPoses)));

    if(ctl().t() + config_.zmpHorizon <= focusedFootstep.transitEndTime)
    {
      break;
    }
  }

  if(footstepQueue_.empty() || footstepQueue_.back().transitEndTime < ctl().t() + config_.zmpHorizon)
  {
    // Keep ZMP at the foot middle position
    zmpTrajFunc_->appendPoint(std::make_pair(ctl().t() + config_.zmpHorizon, calcZmpWithOffset(footPoses)));
  }

  zmpTrajFunc_->calcCoeff();
}

Eigen::Vector3d FootManager::calcZmpWithOffset(const Foot & foot, const sva::PTransformd & footPose) const
{
  Eigen::Vector3d zmpOffset = config_.zmpOffset;
  if(foot == Foot::Right)
  {
    zmpOffset.y() *= -1;
  }
  return (sva::PTransformd(zmpOffset) * footPose).translation();
}

Eigen::Vector3d FootManager::calcZmpWithOffset(const std::unordered_map<Foot, sva::PTransformd> & footPoses) const
{
  if(footPoses.size() == 0)
  {
    mc_rtc::log::error("[FootManager] footPoses is empty in zmpWithOffset.");
    return Eigen::Vector3d::Zero();
  }
  else if(footPoses.size() == 1)
  {
    return calcZmpWithOffset(footPoses.begin()->first, footPoses.begin()->second);
  }
  else //  if(footPoses.size() == 2)
  {
    return 0.5
           * (calcZmpWithOffset(Foot::Left, footPoses.at(Foot::Left))
              + calcZmpWithOffset(Foot::Right, footPoses.at(Foot::Right)));
  }
}

double FootManager::touchDownRemainingDuration() const
{
  if(supportPhase_ == SupportPhase::DoubleSupport)
  {
    return 0;
  }
  else
  {
    return swingFootstep_->swingEndTime - ctl().t();
  }
}

bool FootManager::detectTouchDown() const
{
  // False for double support phase
  if(supportPhase_ == SupportPhase::DoubleSupport)
  {
    return false;
  }

  // False if the remaining duration does not meet the threshold
  if(touchDownRemainingDuration() > config_.touchDownRemainingDuration)
  {
    return false;
  }

  // False if the position error does not meet the threshold
  Foot swingFoot = (supportPhase_ == SupportPhase::LeftSupport ? Foot::Right : Foot::Left);
  if((swingFootstep_->pose.translation() - ctl().footTasks_.at(swingFoot)->surfacePose().translation()).norm()
     > config_.touchDownPosError)
  {
    return false;
  }

  // False if the normal force does not meet the threshold
  double fz = ctl().robot().surfaceWrench(surfaceName(swingFoot)).force().z();
  if(fz < config_.touchDownForceZ)
  {
    return false;
  }

  return true;
}

std::string std::to_string(const Foot & foot)
{
  if(foot == Foot::Left)
  {
    return std::string("Left");
  }
  else if(foot == Foot::Right)
  {
    return std::string("Right");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported foot: {}", std::to_string(static_cast<int>(foot)));
  }
}

std::string std::to_string(const SupportPhase & phase)
{
  if(phase == SupportPhase::DoubleSupport)
  {
    return std::string("DoubleSupport");
  }
  else if(phase == SupportPhase::LeftSupport)
  {
    return std::string("LeftSupport");
  }
  else if(phase == SupportPhase::RightSupport)
  {
    return std::string("RightSupport");
  }
  else
  {
    mc_rtc::log::error_and_throw("[to_string] Unsupported support phase: {}", std::to_string(static_cast<int>(phase)));
  }
}

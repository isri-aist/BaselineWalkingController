#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/CoMTask.h>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/tasks/FirstOrderImpedanceTask.h>
#include <BaselineWalkingController/wrench/Contact.h>
#include <BaselineWalkingController/wrench/WrenchDistribution.h>

using namespace BWC;

void CentroidalManager::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("name", name);
  mcRtcConfig("method", method);
  mcRtcConfig("useActualStateForMpc", useActualStateForMpc);
  mcRtcConfig("enableZmpFeedback", enableZmpFeedback);
  mcRtcConfig("enableComZFeedback", enableComZFeedback);
  mcRtcConfig("dcmGainP", dcmGainP);
  mcRtcConfig("zmpVelGain", zmpVelGain);
  mcRtcConfig("comZGainP", comZGainP);
  mcRtcConfig("comZGainD", comZGainD);
  mcRtcConfig("refComZ", refComZ);
  mcRtcConfig("useTargetPoseForControlRobotAnchorFrame", useTargetPoseForControlRobotAnchorFrame);
  mcRtcConfig("useActualComForWrenchDist", useActualComForWrenchDist);
  mcRtcConfig("wrenchDistConfig", wrenchDistConfig);
}

CentroidalManager::CentroidalManager(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig)
: ctlPtr_(ctlPtr)
{
}

void CentroidalManager::reset()
{
  robotMass_ = ctl().robot().mass();
}

void CentroidalManager::update()
{
  // Set MPC state
  if(config().useActualStateForMpc)
  {
    mpcCom_ = ctl().realRobot().com();
    mpcComVel_ = ctl().realRobot().comVelocity();
  }
  else
  {
    // Task targets are the planned state in the previous step
    mpcCom_ = ctl().comTask_->com();
    mpcComVel_ = ctl().comTask_->refVel();
  }

  // Run MPC
  runMpc();

  // Calculate target wrench
  {
    controlZmp_ = plannedZmp_;
    controlForceZ_ = plannedForceZ_;

    // Compensate ZMP delay
    // See equation (10) of https://ieeexplore.ieee.org/abstract/document/6094838
    Eigen::Vector3d refZmpVel = ctl().footManager_->calcRefZmp(ctl().t(), 1);
    controlZmp_.head<2>() += config().zmpVelGain * refZmpVel.head<2>();

    // Apply DCM feedback
    if(config().enableZmpFeedback)
    {
      double omega = std::sqrt(plannedForceZ_ / (robotMass_ * mpcCom_.z()));
      Eigen::Vector3d plannedDcm = ctl().comTask_->com() + ctl().comTask_->refVel() / omega;
      Eigen::Vector3d actualDcm = ctl().realRobot().com() + ctl().realRobot().comVelocity() / omega;
      controlZmp_.head<2>() += config().dcmGainP * (actualDcm - plannedDcm).head<2>();
    }

    // Apply ForceZ feedback
    if(config().enableComZFeedback)
    {
      double plannedComZ = ctl().comTask_->com().z();
      double actualComZ = ctl().realRobot().com().z();
      double plannedComVelZ = ctl().comTask_->refVel().z();
      double actualComVelZ = ctl().realRobot().comVelocity().z();
      controlForceZ_ -=
          config().comZGainP * (actualComZ - plannedComZ) + config().comZGainD * (actualComVelZ - plannedComVelZ);
    }

    // Convert ZMP to wrench and distribute
    contactList_ = ctl().footManager_->calcContactList();
    if(!wrenchDist_ || wrenchDist_->contactList_ != contactList_)
    {
      wrenchDist_ = std::make_shared<WrenchDistribution>(contactList_, config().wrenchDistConfig);
    }
    Eigen::Vector3d comForWrenchDist =
        (config().useActualComForWrenchDist ? ctl().realRobot().com() : ctl().comTask_->com());
    sva::ForceVecd controlWrench;
    controlWrench.force() << controlForceZ_ / comForWrenchDist.z()
                                 * (comForWrenchDist.head<2>() - controlZmp_.head<2>()),
        controlForceZ_;
    controlWrench.moment().setZero(); // Moment is represented around CoM
    wrenchDist_->run(controlWrench, comForWrenchDist);
  }

  // Set target of tasks
  {
    // Set target of CoM task
    Eigen::Vector3d plannedComAccel;
    plannedComAccel << plannedForceZ_ / (robotMass_ * mpcCom_.z()) * (mpcCom_.head<2>() - plannedZmp_.head<2>()),
        plannedForceZ_ / robotMass_;
    plannedComAccel.z() -= CCC::constants::g;
    Eigen::Vector3d nextPlannedCom =
        mpcCom_ + ctl().dt() * mpcComVel_ + 0.5 * std::pow(ctl().dt(), 2) * plannedComAccel;
    Eigen::Vector3d nextPlannedComVel = mpcComVel_ + ctl().dt() * plannedComAccel;
    if(isConstantComZ())
    {
      nextPlannedCom.z() = config().refComZ + ctl().footManager_->calcRefGroundPosZ(ctl().t());
      nextPlannedComVel.z() = ctl().footManager_->calcRefGroundPosZ(ctl().t(), 1);
      plannedComAccel.z() = ctl().footManager_->calcRefGroundPosZ(ctl().t(), 2);
    }
    ctl().comTask_->com(nextPlannedCom);
    ctl().comTask_->refVel(nextPlannedComVel);
    ctl().comTask_->refAccel(plannedComAccel);

    // Set target wrench of foot tasks
    const auto & targetWrenchList = wrenchDist_->calcWrenchList();
    for(const auto & foot : Feet::Both)
    {
      sva::ForceVecd targetWrench = sva::ForceVecd::Zero();
      if(targetWrenchList.count(foot))
      {
        targetWrench = targetWrenchList.at(foot);
      }
      ctl().footTasks_.at(foot)->targetWrenchW(targetWrench);
    }
  }
}

void CentroidalManager::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"BWC", config().name}, mc_rtc::gui::Label("method", [this]() { return config().method; }),
      mc_rtc::gui::Checkbox(
          "useActualStateForMpc", [this]() { return config().useActualStateForMpc; },
          [this]() { config().useActualStateForMpc = !config().useActualStateForMpc; }),
      mc_rtc::gui::Checkbox(
          "enableZmpFeedback", [this]() { return config().enableZmpFeedback; },
          [this]() { config().enableZmpFeedback = !config().enableZmpFeedback; }),
      mc_rtc::gui::Checkbox(
          "enableComZFeedback", [this]() { return config().enableComZFeedback; },
          [this]() { config().enableComZFeedback = !config().enableComZFeedback; }),
      mc_rtc::gui::NumberInput(
          "dcmGainP", [this]() { return config().dcmGainP; }, [this](double v) { config().dcmGainP = v; }),
      mc_rtc::gui::NumberInput(
          "zmpVelGain", [this]() { return config().zmpVelGain; }, [this](double v) { config().zmpVelGain = v; }),
      mc_rtc::gui::NumberInput(
          "comZGainP", [this]() { return config().comZGainP; }, [this](double v) { config().comZGainP = v; }),
      mc_rtc::gui::NumberInput(
          "comZGainD", [this]() { return config().comZGainD; }, [this](double v) { config().comZGainD = v; }),
      mc_rtc::gui::NumberInput(
          "refComZ", [this]() { return config().refComZ; }, [this](double v) { config().refComZ = v; }),
      mc_rtc::gui::Checkbox(
          "useTargetPoseForControlRobotAnchorFrame",
          [this]() { return config().useTargetPoseForControlRobotAnchorFrame; },
          [this]() {
            config().useTargetPoseForControlRobotAnchorFrame = !config().useTargetPoseForControlRobotAnchorFrame;
          }),
      mc_rtc::gui::Checkbox(
          "useActualComForWrenchDist", [this]() { return config().useActualComForWrenchDist; },
          [this]() { config().useActualComForWrenchDist = !config().useActualComForWrenchDist; }));
}

void CentroidalManager::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"BWC", config().name});
}

void CentroidalManager::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(config().name + "_Config_method", this, [this]() { return config().method; });
  logger.addLogEntry(config().name + "_Config_useActualStateForMpc", this,
                     [this]() { return config().useActualStateForMpc; });
  logger.addLogEntry(config().name + "_Config_enableZmpFeedback", this,
                     [this]() { return config().enableZmpFeedback; });
  logger.addLogEntry(config().name + "_Config_enableComZFeedback", this,
                     [this]() { return config().enableComZFeedback; });
  logger.addLogEntry(config().name + "_Config_dcmGainP", this, [this]() { return config().dcmGainP; });
  logger.addLogEntry(config().name + "_Config_zmpVelGain", this, [this]() { return config().zmpVelGain; });
  logger.addLogEntry(config().name + "_Config_comZGainP", this, [this]() { return config().comZGainP; });
  logger.addLogEntry(config().name + "_Config_comZGainD", this, [this]() { return config().comZGainD; });
  logger.addLogEntry(config().name + "_Config_refComZ", this, [this]() { return config().refComZ; });
  logger.addLogEntry(config().name + "_Config_useTargetPoseForControlRobotAnchorFrame", this,
                     [this]() { return config().useTargetPoseForControlRobotAnchorFrame; });
  logger.addLogEntry(config().name + "_Config_useActualComForWrenchDist", this,
                     [this]() { return config().useActualComForWrenchDist; });

  MC_RTC_LOG_HELPER(config().name + "_CoM_MPC", mpcCom_);
  logger.addLogEntry(config().name + "_CoM_planned", this, [this]() { return ctl().comTask_->com(); });
  logger.addLogEntry(config().name + "_CoM_controlRobot", this, [this]() { return ctl().robot().com(); });
  logger.addLogEntry(config().name + "_CoM_realRobot", this, [this]() { return ctl().realRobot().com(); });

  MC_RTC_LOG_HELPER(config().name + "_forceZ_planned", plannedForceZ_);
  MC_RTC_LOG_HELPER(config().name + "_forceZ_control", controlForceZ_);

  logger.addLogEntry(config().name + "_ZMP_ref", this, [this]() { return ctl().footManager_->calcRefZmp(ctl().t()); });
  MC_RTC_LOG_HELPER(config().name + "_ZMP_planned", plannedZmp_);
  MC_RTC_LOG_HELPER(config().name + "_ZMP_control", controlZmp_);
  logger.addLogEntry(config().name + "_ZMP_controlWrenchDist", this, [this]() {
    return wrenchDist_ ? calcZmp(wrenchDist_->calcWrenchList()) : Eigen::Vector3d::Zero();
  });
  logger.addLogEntry(config().name + "_ZMP_measured", this, [this]() {
    std::unordered_map<Foot, sva::ForceVecd> sensorWrenchList;
    for(const auto & foot : ctl().footManager_->getContactFeet())
    {
      const auto & surfaceName = ctl().footManager_->surfaceName(foot);
      const auto & sensorName = ctl().robot().indirectSurfaceForceSensor(surfaceName).name();
      const auto & sensor = ctl().robot().forceSensor(sensorName);
      const auto & sensorWrench = sensor.worldWrenchWithoutGravity(ctl().robot());
      sensorWrenchList.emplace(foot, sensorWrench);
    }
    return calcZmp(sensorWrenchList);
  });
  logger.addLogEntry(config().name + "_ZMP_SupportRegion_min", this, [this]() {
    Eigen::Vector2d minPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
    for(const auto & contactKV : contactList_)
    {
      for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
      {
        minPos = minPos.cwiseMin(vertexWithRidge.vertex.head<2>());
      }
    }
    return minPos;
  });
  logger.addLogEntry(config().name + "_ZMP_SupportRegion_max", this, [this]() {
    Eigen::Vector2d maxPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());
    for(const auto & contactKV : contactList_)
    {
      for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
      {
        maxPos = maxPos.cwiseMax(vertexWithRidge.vertex.head<2>());
      }
    }
    return maxPos;
  });
}

void CentroidalManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}

void CentroidalManager::setAnchorFrame()
{
  std::string anchorName = "KinematicAnchorFrame::" + ctl().robot().name();
  if(ctl().datastore().has(anchorName))
  {
    ctl().datastore().remove(anchorName);
  }
  ctl().datastore().make_call(anchorName, [this](const mc_rbdyn::Robot & robot) { return calcAnchorFrame(robot); });
}

sva::PTransformd CentroidalManager::calcAnchorFrame(const mc_rbdyn::Robot & robot) const
{
  double leftFootSupportRatio = ctl().footManager_->leftFootSupportRatio();
  bool isControlRobot = (&(ctl().robot()) == &robot);

  if(isControlRobot && config().useTargetPoseForControlRobotAnchorFrame)
  {
    return sva::interpolate(ctl().footManager_->targetFootPose(Foot::Right),
                            ctl().footManager_->targetFootPose(Foot::Left), leftFootSupportRatio);
  }
  else
  {
    return sva::interpolate(robot.surfacePose(ctl().footManager_->surfaceName(Foot::Right)),
                            robot.surfacePose(ctl().footManager_->surfaceName(Foot::Left)), leftFootSupportRatio);
  }
}

Eigen::Vector3d CentroidalManager::calcZmp(const std::unordered_map<Foot, sva::ForceVecd> & wrenchList,
                                           double zmpPlaneHeight,
                                           const Eigen::Vector3d & zmpPlaneNormal) const
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();
  for(const auto & wrenchKV : wrenchList)
  {
    totalWrench += wrenchKV.second;
  }

  Eigen::Vector3d zmpPlaneOrigin = Eigen::Vector3d(0, 0, zmpPlaneHeight);
  Eigen::Vector3d zmp = zmpPlaneOrigin;

  if(totalWrench.force().z() > 0)
  {
    Eigen::Vector3d momentInZmpPlane = totalWrench.moment() - zmpPlaneOrigin.cross(totalWrench.force());
    zmp += zmpPlaneNormal.cross(momentInZmpPlane) / totalWrench.force().z();
  }

  return zmp;
}

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerFootGuidedControl.h>

using namespace BWC;

void CentroidalManagerFootGuidedControl::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("reinitForRefComZ", reinitForRefComZ);
}

CentroidalManagerFootGuidedControl::CentroidalManagerFootGuidedControl(BaselineWalkingController * ctlPtr,
                                                                       const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerFootGuidedControl::reset()
{
  CentroidalManager::reset();

  footGuided_ = std::make_shared<CCC::FootGuidedControl>(config_.refComZ);
  lastRefComZ_ = config_.refComZ;
}

void CentroidalManagerFootGuidedControl::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_FootGuided_capturePoint", this, [this]() -> Eigen::Vector2d {
    return mpcCom_.head<2>() + std::sqrt(lastRefComZ_ / CCC::constants::g) * mpcComVel_.head<2>();
  });
}

void CentroidalManagerFootGuidedControl::runMpc()
{
  double refComZ = calcRefComZ(ctl().t());
  if(refComZ != lastRefComZ_)
  {
    if(config_.reinitForRefComZ)
    {
      footGuided_ = std::make_shared<CCC::FootGuidedControl>(refComZ);
    }
    lastRefComZ_ = refComZ;
  }

  CCC::FootGuidedControl::InitialParam initialParam =
      mpcCom_.head<2>() + std::sqrt(refComZ / CCC::constants::g) * mpcComVel_.head<2>();
  CCC::FootGuidedControl::RefData refData = calcRefData();

  Eigen::Vector2d plannedData = footGuided_->planOnce(refData, initialParam, ctl().t());
  plannedZmp_ << plannedData, refZmp_.z();
  plannedForceZ_ = robotMass_ * CCC::constants::g;
}

CCC::FootGuidedControl::RefData CentroidalManagerFootGuidedControl::calcRefData() const
{
  double constantZmpDuration = 1.0; // [sec]
  double footstepsMergeDurationThre = 0.1; // [sec]
  double horizonMargin = 1e-2; // [sec]
  CCC::FootGuidedControl::RefData refData;

  if(ctl().footManager_->footstepQueue().empty())
  {
    refData.transit_start_zmp =
        ctl()
            .footManager_
            ->calcZmpWithOffset({{Foot::Left, ctl().footManager_->targetFootPose(Foot::Left)},
                                 {Foot::Right, ctl().footManager_->targetFootPose(Foot::Right)}})
            .head<2>();
    refData.transit_end_zmp = refData.transit_start_zmp;
    refData.transit_start_time = ctl().t() + constantZmpDuration;
    refData.transit_duration = 0;
  }
  else
  {
    const auto & footstep = ctl().footManager_->footstepQueue().front();
    if(ctl().t() < footstep.swingStartTime)
    {
      refData.transit_start_zmp =
          ctl()
              .footManager_
              ->calcZmpWithOffset({{Foot::Left, ctl().footManager_->targetFootPose(Foot::Left)},
                                   {Foot::Right, ctl().footManager_->targetFootPose(Foot::Right)}})
              .head<2>();
      refData.transit_end_zmp =
          ctl()
              .footManager_
              ->calcZmpWithOffset(opposite(footstep.foot), ctl().footManager_->targetFootPose(opposite(footstep.foot)))
              .head<2>();
      refData.transit_start_time = footstep.transitStartTime;
      refData.transit_duration = footstep.swingStartTime - footstep.transitStartTime;

      // If the double support duration is short, concatenate the previous and current footsteps to avoid the horizon
      // becoming too short
      if(ctl().footManager_->prevFootstep())
      {
        const auto & prevFootstep = ctl().footManager_->prevFootstep();
        if(prevFootstep->foot == opposite(footstep.foot)
           && footstep.transitStartTime - prevFootstep->transitEndTime < footstepsMergeDurationThre)
        {
          refData.transit_start_zmp =
              ctl()
                  .footManager_->calcZmpWithOffset(footstep.foot, ctl().footManager_->targetFootPose(footstep.foot))
                  .head<2>();
          refData.transit_start_time = prevFootstep->swingEndTime;
          refData.transit_duration = footstep.swingStartTime - prevFootstep->swingEndTime;
        }
      }
    }
    else
    {
      refData.transit_start_zmp =
          ctl()
              .footManager_
              ->calcZmpWithOffset(opposite(footstep.foot), ctl().footManager_->targetFootPose(opposite(footstep.foot)))
              .head<2>();
      refData.transit_end_zmp = ctl()
                                    .footManager_
                                    ->calcZmpWithOffset({{opposite(footstep.foot),
                                                          ctl().footManager_->targetFootPose(opposite(footstep.foot))},
                                                         {footstep.foot, footstep.pose}})
                                    .head<2>();
      refData.transit_start_time = footstep.swingEndTime;
      refData.transit_duration = footstep.transitEndTime - footstep.swingEndTime;

      // If the double support duration is short, concatenate the current and next footsteps to avoid the horizon
      // becoming too short
      if(ctl().footManager_->footstepQueue().size() >= 2)
      {
        const auto & nextFootstep = ctl().footManager_->footstepQueue()[1];
        if(nextFootstep.foot == opposite(footstep.foot)
           && nextFootstep.transitStartTime - footstep.transitEndTime < footstepsMergeDurationThre)
        {
          refData.transit_end_zmp = ctl().footManager_->calcZmpWithOffset(footstep.foot, footstep.pose).head<2>();
          refData.transit_duration = nextFootstep.swingStartTime - footstep.swingEndTime;
        }
      }
    }

    // Ensure a horizon, since a horizon close to zero produces a very large input
    if(refData.transit_start_time + refData.transit_duration < ctl().t() + horizonMargin)
    {
      refData.transit_duration = ctl().t() + horizonMargin - refData.transit_start_time;
    }
  }

  return refData;
}

#include <functional>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerPreviewControlZmp.h>

using namespace BWC;

void CentroidalManagerPreviewControlZmp::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  mcRtcConfig("reinitForRefComZ", reinitForRefComZ);
}

CentroidalManagerPreviewControlZmp::CentroidalManagerPreviewControlZmp(BaselineWalkingController * ctlPtr,
                                                                       const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerPreviewControlZmp::reset()
{
  CentroidalManager::reset();

  pc_ = std::make_shared<CCC::PreviewControlZmp>(config_.refComZ, config_.horizonDuration, config_.horizonDt);
  lastRefComZ_ = config_.refComZ;
  firstIter_ = true;
}

void CentroidalManagerPreviewControlZmp::runMpc()
{
  double refComZ = calcRefComZ(ctl().t());
  constexpr double threRefComZ = 1e-3; // [m]
  if(std::abs(refComZ - lastRefComZ_) > threRefComZ)
  {
    if(config_.reinitForRefComZ)
    {
      pc_ = std::make_shared<CCC::PreviewControlZmp>(refComZ, config_.horizonDuration, config_.horizonDt);
    }
    lastRefComZ_ = refComZ;
  }

  CCC::PreviewControlZmp::InitialParam initialParam;
  initialParam.pos = mpcCom_.head<2>();
  initialParam.vel = mpcComVel_.head<2>();
  if(firstIter_)
  {
    initialParam.acc.setZero();
  }
  else
  {
    // Since the actual CoM acceleration cannot be obtained, the CoM acceleration is always calculated from LIPM dynamics
    initialParam.acc = CCC::constants::g / refComZ * (mpcCom_ - plannedZmp_).head<2>();
  }

  Eigen::Vector2d plannedData =
      pc_->planOnce(std::bind(&CentroidalManagerPreviewControlZmp::calcRefData, this, std::placeholders::_1),
                    initialParam, ctl().t(), ctl().dt());
  plannedZmp_ << plannedData, refZmp_.z();
  plannedForceZ_ = robotMass_ * CCC::constants::g;

  if(firstIter_)
  {
    firstIter_ = false;
  }
}

Eigen::Vector2d CentroidalManagerPreviewControlZmp::calcRefData(double t) const
{
  return ctl().footManager_->calcRefZmp(t).head<2>();
}

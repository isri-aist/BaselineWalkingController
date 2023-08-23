#include <functional>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/RobotUtils.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerIntrinsicallyStableMpc.h>

using namespace BWC;

void CentroidalManagerIntrinsicallyStableMpc::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  if(mcRtcConfig.has("qpSolverType"))
  {
    qpSolverType = QpSolverCollection::strToQpSolverType(mcRtcConfig("qpSolverType"));
  }
  mcRtcConfig("reinitForRefComZ", reinitForRefComZ);
}

CentroidalManagerIntrinsicallyStableMpc::CentroidalManagerIntrinsicallyStableMpc(
    BaselineWalkingController * ctlPtr,
    const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerIntrinsicallyStableMpc::reset()
{
  CentroidalManager::reset();

  mpc_ = std::make_shared<CCC::IntrinsicallyStableMpc>(config_.refComZ, config_.horizonDuration, config_.horizonDt,
                                                       config_.qpSolverType);
  lastRefComZ_ = config_.refComZ;
  firstIter_ = true;
}

void CentroidalManagerIntrinsicallyStableMpc::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_IntrinsicallyStableMpc_zmpLimits_min", this,
                     [this]() { return calcRefData(ctl().t()).zmp_limits[0]; });
  logger.addLogEntry(config_.name + "_IntrinsicallyStableMpc_zmpLimits_max", this,
                     [this]() { return calcRefData(ctl().t()).zmp_limits[1]; });
}

void CentroidalManagerIntrinsicallyStableMpc::runMpc()
{
  double refComZ = calcRefComZ(ctl().t());
  if(refComZ != lastRefComZ_)
  {
    if(config_.reinitForRefComZ)
    {
      mpc_ = std::make_shared<CCC::IntrinsicallyStableMpc>(refComZ, config_.horizonDuration, config_.horizonDt,
                                                           config_.qpSolverType);
    }
    lastRefComZ_ = refComZ;
  }

  CCC::IntrinsicallyStableMpc::InitialParam initialParam;
  initialParam.capture_point = mpcCom_.head<2>() + std::sqrt(refComZ / CCC::constants::g) * mpcComVel_.head<2>();
  if(firstIter_)
  {
    initialParam.planned_zmp = mpcCom_.head<2>();
  }
  else
  {
    initialParam.planned_zmp = plannedZmp_.head<2>();
  }

  Eigen::Vector2d plannedData =
      mpc_->planOnce(std::bind(&CentroidalManagerIntrinsicallyStableMpc::calcRefData, this, std::placeholders::_1),
                     initialParam, ctl().t(), ctl().dt());
  plannedZmp_ << plannedData, refZmp_.z();
  plannedForceZ_ = robotMass_ * CCC::constants::g;

  if(firstIter_)
  {
    firstIter_ = false;
  }
}

CCC::IntrinsicallyStableMpc::RefData CentroidalManagerIntrinsicallyStableMpc::calcRefData(double t) const
{
  CCC::IntrinsicallyStableMpc::RefData refData;
  refData.zmp = ctl().footManager_->calcRefZmp(t).head<2>();
  Eigen::Vector2d minPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector2d maxPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());
  for(const auto & footPoseKV : ctl().footManager_->calcContactFootPoses(t))
  {
    const auto & surface = ctl().robot().surface(ctl().footManager_->surfaceName(footPoseKV.first));
    for(const auto & pos : calcSurfaceVertexList(surface, footPoseKV.second))
    {
      minPos = minPos.cwiseMin(pos.head<2>());
      maxPos = maxPos.cwiseMax(pos.head<2>());
    }
  }
  refData.zmp_limits[0] = minPos;
  refData.zmp_limits[1] = maxPos;
  return refData;
}

#include <functional>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerIntrinsicallyStableMpc.h>
#include <BaselineWalkingController/wrench/Contact.h>

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

  firstIter_ = true;
}

void CentroidalManagerIntrinsicallyStableMpc::runMpc()
{
  CCC::IntrinsicallyStableMpc::InitialParam initialParam;
  initialParam.capture_point =
      mpcCom_.head<2>() + std::sqrt(config_.refComZ / CCC::constants::g) * mpcComVel_.head<2>();
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
  // \todo
  // Eigen::Vector2d minPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::max());
  // Eigen::Vector2d maxPos = Eigen::Vector2d::Constant(std::numeric_limits<double>::lowest());
  // for(const auto & contactKV : contactList_)
  // {
  //   for(const auto & vertexWithRidge : contactKV.second->vertexWithRidgeList_)
  //   {
  //     minPos = minPos.cwiseMin(vertexWithRidge.vertex.head<2>());
  //     maxPos = maxPos.cwiseMax(vertexWithRidge.vertex.head<2>());
  //   }
  // }
  // refData.zmp_limits[0] = minPos;
  // refData.zmp_limits[1] = maxPos;
  refData.zmp_limits[0] = refData.zmp + Eigen::Vector2d::Constant(-0.2);
  refData.zmp_limits[1] = refData.zmp + Eigen::Vector2d::Constant(0.2);
  return refData;
};

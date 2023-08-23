#include <functional>

#include <CCC/Constants.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/centroidal/CentroidalManagerDdpZmp.h>

using namespace BWC;

void CentroidalManagerDdpZmp::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  CentroidalManager::Configuration::load(mcRtcConfig);

  mcRtcConfig("horizonDuration", horizonDuration);
  mcRtcConfig("horizonDt", horizonDt);
  mcRtcConfig("ddpMaxIter", ddpMaxIter);
  if(mcRtcConfig.has("mpcWeightParam"))
  {
    mcRtcConfig("mpcWeightParam")("runningComPosZ", mpcWeightParam.running_com_pos_z);
    mcRtcConfig("mpcWeightParam")("runningZmp", mpcWeightParam.running_zmp);
    mcRtcConfig("mpcWeightParam")("runningForceZ", mpcWeightParam.running_force_z);
    mcRtcConfig("mpcWeightParam")("terminalComPosXy", mpcWeightParam.terminal_com_pos_xy);
    mcRtcConfig("mpcWeightParam")("terminalComPosZ", mpcWeightParam.terminal_com_pos_z);
    mcRtcConfig("mpcWeightParam")("terminalComVel", mpcWeightParam.terminal_com_vel);
  }
}

CentroidalManagerDdpZmp::CentroidalManagerDdpZmp(BaselineWalkingController * ctlPtr,
                                                 const mc_rtc::Configuration & mcRtcConfig)
: CentroidalManager(ctlPtr, mcRtcConfig)
{
  config_.load(mcRtcConfig);
}

void CentroidalManagerDdpZmp::reset()
{
  CentroidalManager::reset();

  ddp_ = std::make_shared<CCC::DdpZmp>(robotMass_, config_.horizonDt,
                                       static_cast<int>(std::floor(config_.horizonDuration / config_.horizonDt)),
                                       config_.mpcWeightParam);
  ddp_->ddp_solver_->config().max_iter = config_.ddpMaxIter;
}

void CentroidalManagerDdpZmp::addToLogger(mc_rtc::Logger & logger)
{
  CentroidalManager::addToLogger(logger);

  logger.addLogEntry(config_.name + "_DDP_computationDuration", this,
                     [this]() { return ddp_->ddp_solver_->computationDuration().solve; });
  logger.addLogEntry(config_.name + "_DDP_iter", this, [this]() {
    return ddp_->ddp_solver_->traceDataList().empty() ? 0 : ddp_->ddp_solver_->traceDataList().back().iter;
  });
}

void CentroidalManagerDdpZmp::runMpc()
{
  CCC::DdpZmp::InitialParam initialParam;
  initialParam.pos = mpcCom_;
  initialParam.vel = mpcComVel_;
  if(static_cast<int>(ddp_->ddp_solver_->controlData().u_list.size()) == ddp_->ddp_solver_->config().horizon_steps)
  {
    initialParam.u_list = ddp_->ddp_solver_->controlData().u_list;
  }
  else
  {
    initialParam.u_list.assign(
        ddp_->ddp_solver_->config().horizon_steps,
        CCC::DdpZmp::DdpProblem::InputDimVector(mpcCom_.x(), mpcCom_.y(), robotMass_ * CCC::constants::g));
  }

  CCC::DdpZmp::PlannedData plannedData = ddp_->planOnce(
      std::bind(&CentroidalManagerDdpZmp::calcRefData, this, std::placeholders::_1), initialParam, ctl().t());
  plannedZmp_ << plannedData.zmp, refZmp_.z();
  plannedForceZ_ = plannedData.force_z;
}

CCC::DdpZmp::RefData CentroidalManagerDdpZmp::calcRefData(double t) const
{
  CCC::DdpZmp::RefData refData;
  refData.zmp = ctl().footManager_->calcRefZmp(t);
  refData.com_z = calcRefComZ(t) + ctl().footManager_->calcRefGroundPosZ(t);
  return refData;
}

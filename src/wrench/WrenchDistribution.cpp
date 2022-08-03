#include <BaselineWalkingController/wrench/Contact.h>
#include <BaselineWalkingController/wrench/WrenchDistribution.h>

using namespace BWC;

WrenchDistribution::WrenchDistribution(const std::unordered_map<Foot, std::shared_ptr<Contact>> & contactList,
                                       const mc_rtc::Configuration & mcRtcConfig)
: contactList_(contactList)
{
  int colNum = 0;
  for(const auto & contactKV : contactList_)
  {
    colNum += contactKV.second->graspMat_.cols();
  }
  resultWrenchRatio_ = Eigen::VectorXd::Zero(colNum);

  mcRtcConfig("wrenchWeight", wrenchWeight_);
  mcRtcConfig("regularWeight", regularWeight_);
  mcRtcConfig("ridgeForceMinMax", ridgeForceMinMax_);

  QpSolverCollection::QpSolverType qpSolverType = QpSolverCollection::QpSolverType::Any;
  if(mcRtcConfig.has("qpSolverType"))
  {
    qpSolverType = QpSolverCollection::strToQpSolverType(mcRtcConfig("qpSolverType"));
  }
  qpSolver_ = QpSolverCollection::allocateQpSolver(qpSolverType);
}

sva::ForceVecd WrenchDistribution::run(const sva::ForceVecd & desiredTotalWrench, const Eigen::Vector3d & origin)
{
  desiredTotalWrench_ = desiredTotalWrench;

  // Return if variable dimension is zero
  if(resultWrenchRatio_.size() == 0)
  {
    resultTotalWrench_ = sva::ForceVecd::Zero();
    return resultTotalWrench_;
  }

  // Construct totalGraspMat
  Eigen::Matrix<double, 6, Eigen::Dynamic> totalGraspMat(6, resultWrenchRatio_.size());
  {
    int colNum = 0;
    for(const auto & contactKV : contactList_)
    {
      totalGraspMat.middleCols(colNum, contactKV.second->graspMat_.cols()) = contactKV.second->graspMat_;
      colNum += contactKV.second->graspMat_.cols();
    }
    if(origin.norm() > 0)
    {
      for(int i = 0; i < colNum; i++)
      {
        // totalGraspMat.col(i).tail<3>() is the force ridge
        totalGraspMat.col(i).head<3>() -= origin.cross(totalGraspMat.col(i).tail<3>());
      }
    }
  }

  // Solve QP
  {
    int varDim = resultWrenchRatio_.size();
    if(qpCoeff_.dim_var_ != varDim)
    {
      qpCoeff_.setup(varDim, 0, 0);
    }
    Eigen::MatrixXd weightMat = wrenchWeight_.vector().asDiagonal();
    qpCoeff_.obj_mat_.noalias() = totalGraspMat.transpose() * weightMat * totalGraspMat;
    qpCoeff_.obj_mat_.diagonal().array() += regularWeight_;
    qpCoeff_.obj_vec_.noalias() = -1 * totalGraspMat.transpose() * weightMat * desiredTotalWrench_.vector();
    qpCoeff_.x_min_.setConstant(varDim, ridgeForceMinMax_.first);
    qpCoeff_.x_max_.setConstant(varDim, ridgeForceMinMax_.second);
    resultWrenchRatio_ = qpSolver_->solve(qpCoeff_);
  }

  resultTotalWrench_ = sva::ForceVecd(totalGraspMat * resultWrenchRatio_);
  totalWrenchError_ = resultTotalWrench_ - desiredTotalWrench_;

  return resultTotalWrench_;
}

std::unordered_map<Foot, sva::ForceVecd> WrenchDistribution::calcWrenchList(const Eigen::Vector3d momentOrigin) const
{
  std::unordered_map<Foot, sva::ForceVecd> wrenchList;
  int wrenchRatioIdx = 0;

  for(const auto & contactKV : contactList_)
  {
    wrenchList.emplace(
        contactKV.first,
        contactKV.second->calcWrench(resultWrenchRatio_.segment(wrenchRatioIdx, contactKV.second->graspMat_.cols()),
                                     momentOrigin));
    wrenchRatioIdx += contactKV.second->graspMat_.cols();
  }
  return wrenchList;
}

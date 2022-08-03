#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <BaselineWalkingController/wrench/Contact.h>

using namespace BWC;

FrictionPyramid::FrictionPyramid(double fricCoeff, int divideNum)
{
  for(int i = 0; i < divideNum; i++)
  {
    double theta = 2 * mc_rtc::constants::PI * (static_cast<double>(i) / divideNum);
    localRidgeList_.push_back(
        Eigen::Vector3d(fricCoeff * std::cos(theta), fricCoeff * std::sin(theta), 1).normalized());
  }
}

std::vector<Eigen::Vector3d> FrictionPyramid::calcGlobalRidgeList(const Eigen::Matrix3d & rot) const
{
  std::vector<Eigen::Vector3d> globalRidgeList;
  for(const auto & localRidge : localRidgeList_)
  {
    globalRidgeList.push_back(rot * localRidge);
  }
  return globalRidgeList;
}

Contact::Contact(const std::string & name,
                 double fricCoeff,
                 const std::vector<Eigen::Vector3d> & localVertexList,
                 const sva::PTransformd & pose)
: name_(name)
{
  // Set graspMat_
  FrictionPyramid fricPyramid(fricCoeff);

  graspMat_.resize(6, localVertexList.size() * fricPyramid.divideNum());

  const auto & globalRidgeList = fricPyramid.calcGlobalRidgeList(pose.rotation().transpose());

  for(int vertexIdx = 0; vertexIdx < localVertexList.size(); vertexIdx++)
  {
    Eigen::Vector3d globalVertex = (sva::PTransformd(localVertexList[vertexIdx]) * pose).translation();

    for(int ridgeIdx = 0; ridgeIdx < globalRidgeList.size(); ridgeIdx++)
    {
      const auto & globalRidge = globalRidgeList[ridgeIdx];
      // The top 3 rows are moment, the bottom 3 rows are force.
      graspMat_.col(vertexIdx * fricPyramid.divideNum() + ridgeIdx) << globalVertex.cross(globalRidge), globalRidge;
    }

    vertexWithRidgeList_.push_back(VertexWithRidge(globalVertex, globalRidgeList));
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> localGraspMat(3, fricPyramid.divideNum());
  for(int ridgeIdx = 0; ridgeIdx < fricPyramid.divideNum(); ridgeIdx++)
  {
    localGraspMat.col(ridgeIdx) = fricPyramid.localRidgeList_[ridgeIdx];
  }
  localGraspMatList_.assign(localVertexList.size(), localGraspMat);
}

sva::ForceVecd Contact::calcWrench(const Eigen::VectorXd & wrenchRatio, const Eigen::Vector3d & momentOrigin) const
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();
  int totalIdx = 0;

  for(const auto & vertexWithRidge : vertexWithRidgeList_)
  {
    const Eigen::Vector3d & vertex = vertexWithRidge.vertex;
    const std::vector<Eigen::Vector3d> & ridgeList = vertexWithRidge.ridgeList;

    for(const auto & ridge : ridgeList)
    {
      Eigen::Vector3d force = wrenchRatio(totalIdx) * ridge;
      totalWrench.force() += force;
      totalWrench.moment() += (vertex - momentOrigin).cross(force);
      totalIdx++;
    }
  }

  assert(wrenchRatio.size() == totalIdx);

  return totalWrench;
}

std::vector<Eigen::Vector3d> Contact::calcLocalVertexForceList(const Eigen::VectorXd & wrenchRatio) const
{
  std::vector<Eigen::Vector3d> localForceList;
  int colNum = 0;
  for(const auto & localGraspMat : localGraspMatList_)
  {
    localForceList.push_back(localGraspMat * wrenchRatio.segment(colNum, localGraspMat.cols()));
    colNum += localGraspMat.cols();
  }
  return localForceList;
}

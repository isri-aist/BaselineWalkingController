#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <BaselineWalkingController/wrench/Contact.h>

using namespace BWC;

FrictionPyramid::FrictionPyramid(double fricCoeff, int ridgeNum)
{
  for(int i = 0; i < ridgeNum; i++)
  {
    double theta = 2 * mc_rtc::constants::PI * (static_cast<double>(i) / ridgeNum);
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
  // Set graspMat_ and vertexWithRidgeList_
  FrictionPyramid fricPyramid(fricCoeff);

  graspMat_.resize(6, localVertexList.size() * fricPyramid.ridgeNum());

  const auto & globalRidgeList = fricPyramid.calcGlobalRidgeList(pose.rotation().transpose());

  for(size_t vertexIdx = 0; vertexIdx < localVertexList.size(); vertexIdx++)
  {
    Eigen::Vector3d globalVertex = (sva::PTransformd(localVertexList[vertexIdx]) * pose).translation();

    for(size_t ridgeIdx = 0; ridgeIdx < globalRidgeList.size(); ridgeIdx++)
    {
      const auto & globalRidge = globalRidgeList[ridgeIdx];
      // The top 3 rows are moment, the bottom 3 rows are force.
      graspMat_.col(vertexIdx * fricPyramid.ridgeNum() + ridgeIdx) << globalVertex.cross(globalRidge), globalRidge;
    }

    vertexWithRidgeList_.push_back(VertexWithRidge(globalVertex, globalRidgeList));
  }
}

sva::ForceVecd Contact::calcWrench(const Eigen::VectorXd & wrenchRatio, const Eigen::Vector3d & momentOrigin) const
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();
  int wrenchRatioIdx = 0;

  for(const auto & vertexWithRidge : vertexWithRidgeList_)
  {
    const Eigen::Vector3d & vertex = vertexWithRidge.vertex;
    const std::vector<Eigen::Vector3d> & ridgeList = vertexWithRidge.ridgeList;

    for(const auto & ridge : ridgeList)
    {
      Eigen::Vector3d force = wrenchRatio(wrenchRatioIdx) * ridge;
      totalWrench.force() += force;
      totalWrench.moment() += (vertex - momentOrigin).cross(force);
      wrenchRatioIdx++;
    }
  }

  assert(wrenchRatio.size() == wrenchRatioIdx);

  return totalWrench;
}

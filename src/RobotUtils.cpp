#include <mc_rbdyn/Surface.h>

#include <BaselineWalkingController/RobotUtils.h>

using namespace BWC;

std::vector<Eigen::Vector3d> BWC::calcSurfaceVertexList(const mc_rbdyn::Surface & surface,
                                                        const sva::PTransformd & surfaceOrigin)
{
  std::vector<Eigen::Vector3d> localVertexList;
  for(const auto & point : surface.points())
  {
    // Surface points are represented in body frame, not surface frame
    localVertexList.push_back((point * surface.X_b_s().inv() * surfaceOrigin).translation());
  }
  return localVertexList;
}

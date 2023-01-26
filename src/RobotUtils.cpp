#include <mc_rbdyn/Surface.h>
#include <mc_rtc/Configuration.h>

#include <BaselineWalkingController/RobotUtils.h>

using namespace BWC;

TaskGain::TaskGain(const sva::MotionVecd & _stiffness, const sva::MotionVecd & _damping)
: stiffness(_stiffness), damping(_damping)
{
  if(stiffness.vector().norm() > 0 && damping.vector().norm() == 0)
  {
    damping = sva::MotionVecd(2 * stiffness.vector().cwiseSqrt());
  }
}

TaskGain::TaskGain(const mc_rtc::Configuration & mcRtcConfig)
{
  if(!mcRtcConfig.has("stiffness"))
  {
    mc_rtc::log::error_and_throw("[TaskGain] The stiffness entry in the configuration is mandatory.");
  }

  auto loadMotionVecConfig = [](const mc_rtc::Configuration & motionVecConfig) {
    sva::MotionVecd motionVec;
    if(motionVecConfig.size())
    {
      return static_cast<sva::MotionVecd>(motionVecConfig);
    }
    else
    {
      return sva::MotionVecd(Eigen::Vector6d::Constant(static_cast<double>(motionVecConfig)));
    }
  };
  stiffness = loadMotionVecConfig(mcRtcConfig("stiffness"));
  if(mcRtcConfig.has("damping"))
  {
    damping = loadMotionVecConfig(mcRtcConfig("damping"));
  }
  else
  {
    damping = sva::MotionVecd(2 * stiffness.vector().cwiseSqrt());
  }
}

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

#include <mc_rbdyn/rpy_utils.h>

#include <BaselineWalkingController/MathUtils.h>

using namespace BWC;

sva::PTransformd BWC::projGround(const sva::PTransformd & pose, bool projectZ)
{
  sva::PTransformd projectedPose(sva::RotZ(mc_rbdyn::rpyFromMat(pose.rotation())[2]), pose.translation());
  if(projectZ)
  {
    projectedPose.translation().z() = 0;
  }
  return projectedPose;
}

sva::MotionVecd BWC::projGround(const sva::MotionVecd & vel, bool projectZ)
{
  sva::MotionVecd projectedVel(Eigen::Vector3d(0, 0, vel.angular().z()), vel.linear());
  if(projectZ)
  {
    projectedVel.linear().z() = 0;
  }
  return projectedVel;
}

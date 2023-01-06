#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace BWC
{
/** \brief Calculate the pose projected on to the ground.
    \param pose pose
    \param projectZ true to set the Z position zero (otherwise, keep the original value)
 */
inline sva::PTransformd projGround(const sva::PTransformd & pose, bool projectZ = true)
{
  sva::PTransformd projectedPose(sva::RotZ(mc_rbdyn::rpyFromMat(pose.rotation())[2]), pose.translation());
  if(projectZ)
  {
    projectedPose.translation().z() = 0;
  }
  return projectedPose;
}

/** \brief Calculate the velocity projected on to the ground.
    \param vel velocity
    \param projectZ true to set the Z linear velocity zero (otherwise, keep the original value)
 */
inline sva::MotionVecd projGround(const sva::MotionVecd & vel, bool projectZ = true)
{
  sva::MotionVecd projectedVel(Eigen::Vector3d(0, 0, vel.angular().z()), vel.linear());
  if(projectZ)
  {
    projectedVel.linear().z() = 0;
  }
  return projectedVel;
}
} // namespace BWC

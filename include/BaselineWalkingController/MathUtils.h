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
  Eigen::Vector3d pos = pose.translation();
  if(projectZ)
  {
    pos.z() = 0;
  }
  return sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(pose.rotation())[2]), pos);
}

/** \brief Calculate the velocity projected on to the ground.
    \param vel velocity
    \param projectZ true to set the Z linear velocity zero (otherwise, keep the original value)
 */
inline sva::MotionVecd projGround(const sva::MotionVecd & vel, bool projectZ = true)
{
  Eigen::Vector3d linear = vel.linear();
  if(projectZ)
  {
    linear.z() = 0;
  }
  return sva::MotionVecd(Eigen::Vector3d(0, 0, vel.angular().z()), linear);
}
} // namespace BWC

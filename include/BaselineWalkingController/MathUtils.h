#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace BWC
{
/** \brief Calculate the pose projected on to the ground.
    \param pose pose
    \param projectZ true to set z zero (otherwise, keep the original value)
 */
inline sva::PTransformd projGround(const sva::PTransformd & pose, bool projectZ = true)
{
  Eigen::Vector3d pos = pose.translation();
  if(projectZ)
  {
    pos[2] = 0;
  }
  return sva::PTransformd(sva::RotZ(mc_rbdyn::rpyFromMat(pose.rotation())[2]), pos);
}
} // namespace BWC

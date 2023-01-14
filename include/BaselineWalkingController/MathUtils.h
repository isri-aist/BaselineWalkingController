#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace BWC
{
/** \brief Calculate the pose projected on to the ground.
    \param pose pose
    \param projectZ true to set the Z position zero (otherwise, keep the original value)
 */
sva::PTransformd projGround(const sva::PTransformd & pose, bool projectZ = true);

/** \brief Calculate the velocity projected on to the ground.
    \param vel velocity
    \param projectZ true to set the Z linear velocity zero (otherwise, keep the original value)
 */
sva::MotionVecd projGround(const sva::MotionVecd & vel, bool projectZ = true);
} // namespace BWC

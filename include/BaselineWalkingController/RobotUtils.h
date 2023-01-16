#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rtc
{
class Configuration;
}
namespace mc_rbdyn
{
class Surface;
}

namespace BWC
{
/** \brief IK task gains. */
struct TaskGain
{
  /** \brief Constructor.
      \param _stiffness stiffness
      \param _damping damping (if omitted, the critical damping value will be set)
  */
  TaskGain(const sva::MotionVecd & _stiffness = sva::MotionVecd::Zero(),
           const sva::MotionVecd & _damping = sva::MotionVecd::Zero());

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
  */
  TaskGain(const mc_rtc::Configuration & mcRtcConfig);

  //! Stiffness
  sva::MotionVecd stiffness;

  //! Damping
  sva::MotionVecd damping;
};

/** \brief Calculate the vertices of surface.
    \param surface surface
    \param surfaceOrigin surface origin
 */
std::vector<Eigen::Vector3d> calcSurfaceVertexList(const mc_rbdyn::Surface & surface,
                                                   const sva::PTransformd & surfaceOrigin);
} // namespace BWC

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tasks
{
class TransformTask;
}

namespace BWC
{
struct TaskGain
{
  /** \brief Constructor.
      \param _stiffness stiffness
      \param _damping damping
  */
  TaskGain(const sva::MotionVecd & _stiffness = sva::MotionVecd::Zero(),
           const sva::MotionVecd & _damping = sva::MotionVecd::Zero());

  /** \brief Constructor.
      \param task transform task
  */
  TaskGain(const std::shared_ptr<mc_tasks::TransformTask> & task);

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

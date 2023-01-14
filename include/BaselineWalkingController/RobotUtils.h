#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{
class Surface;
}

namespace BWC
{
/** \brief Calculate the vertices of surface.
    \param surface surface
    \param surfaceOrigin surface origin
 */
std::vector<Eigen::Vector3d> calcSurfaceVertexList(const mc_rbdyn::Surface & surface,
                                                   const sva::PTransformd & surfaceOrigin);
} // namespace BWC

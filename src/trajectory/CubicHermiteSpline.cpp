#include <BaselineWalkingController/trajectory/CubicHermiteSpline.h>

namespace BWC
{
template class CubicHermiteSpline<Eigen::Vector3d>;
template class CubicHermiteSpline<Eigen::VectorXd>;
} // namespace BWC

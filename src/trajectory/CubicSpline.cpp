#include <BaselineWalkingController/trajectory/CubicSpline.h>

namespace BWC
{
template class CubicSpline<Eigen::Vector3d>;
template class CubicSpline<Eigen::VectorXd>;
} // namespace BWC

/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <BaselineWalkingController/trajectory/CubicSpline.h>

TEST(TestCubicSpline, Test1)
{
  std::map<double, Eigen::Vector3d> points = {{10, Eigen::Vector3d(1, 0, 1)},
                                              {11, Eigen::Vector3d(2, 10, 1)},
                                              {13, Eigen::Vector3d(3, -5, 1)},
                                              {15, Eigen::Vector3d(4, -2, 1)},
                                              {20, Eigen::Vector3d(5, 0, 1)}};
  Eigen::Vector3d start_vel(0, 1, -2);
  Eigen::Vector3d end_accel(-5, 100, 0);

  // setup spline
  BWC::CubicSpline<Eigen::Vector3d> sp(
      3, BWC::BoundaryConstraint<Eigen::Vector3d>(BWC::BoundaryConstraintType::Velocity, start_vel),
      BWC::BoundaryConstraint<Eigen::Vector3d>(BWC::BoundaryConstraintType::Acceleration, end_accel), points);
  sp.calcCoeff();

  // check position of waypoints
  for(const auto & point : points)
  {
    double t = point.first;
    EXPECT_TRUE(point.second.isApprox(sp(t)));
  }

  // check continuity of position, velocity, and acceleration
  double t_eps = 1e-8;
  double y_eps = 1e-4;
  double yd_eps = 1e-4;
  double ydd_eps = 1e-4;
  for(auto it = std::next(points.begin()); it != std::prev(points.end()); it++)
  {
    double t = it->first;
    EXPECT_TRUE(((sp(t - t_eps) - sp(t + t_eps)).array() < y_eps).all());
    EXPECT_TRUE(((sp.derivative(t - t_eps, 1) - sp.derivative(t + t_eps, 1)).array() < yd_eps).all());
    EXPECT_TRUE(((sp.derivative(t - t_eps, 2) - sp.derivative(t + t_eps, 2)).array() < ydd_eps).all());
  }

  // check velocity and acceleration of boundary
  EXPECT_TRUE(start_vel.isApprox(sp.derivative(points.begin()->first, 1)));
  EXPECT_TRUE(end_accel.isApprox(sp.derivative(points.rbegin()->first, 2)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

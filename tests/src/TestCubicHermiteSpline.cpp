/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <BaselineWalkingController/trajectory/CubicHermiteSpline.h>
#include <BaselineWalkingController/trajectory/CubicSpline.h>

TEST(TestCubicHermiteSpline, CubicHermiteSpline)
{
  std::map<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>> points = {
      {10, {Eigen::Vector3d(1, 0, 1), Eigen::Vector3d(0, -1, 2)}},
      {11, {Eigen::Vector3d(2, 10, 1), Eigen::Vector3d(-3, 4, 5)}},
      {13, {Eigen::Vector3d(3, -5, 1), Eigen::Vector3d(6, 7, 8)}},
      {15, {Eigen::Vector3d(4, -2, 1), Eigen::Vector3d(-9, -10, -11)}},
      {20, {Eigen::Vector3d(5, 0, 1), Eigen::Vector3d(-2, 1, 0)}}};

  // setup spline
  BWC::CubicHermiteSpline<Eigen::Vector3d> sp(3, points);
  sp.calcCoeff();

  // check position of waypoints
  for(const auto & point : points)
  {
    double t = point.first;
    const Eigen::Vector3d & pos = point.second.first;
    const Eigen::Vector3d & vel = point.second.second;
    EXPECT_TRUE(pos.isApprox(sp(t)));
    EXPECT_TRUE(vel.isApprox(sp.derivative(t, 1)));
  }

  // check continuity of position and velocity
  double t_eps = 1e-8;
  double y_eps = 1e-4;
  double yd_eps = 1e-4;
  for(auto it = std::next(points.begin()); it != std::prev(points.end()); it++)
  {
    double t = it->first;
    EXPECT_TRUE(((sp(t - t_eps) - sp(t + t_eps)).array() < y_eps).all());
    EXPECT_TRUE(((sp.derivative(t - t_eps, 1) - sp.derivative(t + t_eps, 1)).array() < yd_eps).all());
  }
}

TEST(TestCubicHermiteSpline, CompareCubicSplineAndCubicHermiteSpline)
{
  for(int i = 0; i < 100; i++)
  {
    Eigen::Vector2d times = Eigen::Vector2d::Random();
    double start_time = times.minCoeff();
    double end_time = times.maxCoeff();
    Eigen::Vector3d start_pos = Eigen::Vector3d::Random();
    Eigen::Vector3d end_pos = Eigen::Vector3d::Random();
    Eigen::Vector3d start_vel = Eigen::Vector3d::Random();
    Eigen::Vector3d end_vel = Eigen::Vector3d::Random();

    BWC::CubicSpline<Eigen::Vector3d> cubicSpline(
        3, BWC::BoundaryConstraint<Eigen::Vector3d>(BWC::BoundaryConstraintType::Velocity, start_vel),
        BWC::BoundaryConstraint<Eigen::Vector3d>(BWC::BoundaryConstraintType::Velocity, end_vel),
        std::map<double, Eigen::Vector3d>{{start_time, start_pos}, {end_time, end_pos}});
    cubicSpline.calcCoeff();

    BWC::CubicHermiteSpline<Eigen::Vector3d> cubicHermiteSpline(
        3, std::map<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>{{start_time, {start_pos, start_vel}},
                                                                         {end_time, {end_pos, end_vel}}});
    cubicHermiteSpline.calcCoeff();

    const int divideNum = 100;
    for(int j = 0; j <= divideNum; j++)
    {
      double ratio = static_cast<double>(j) / divideNum;
      double t = (1.0 - ratio) * start_time + ratio * end_time;
      EXPECT_TRUE(cubicSpline(t).isApprox(cubicHermiteSpline(t)));
      EXPECT_TRUE(cubicSpline.derivative(t, 1).isApprox(cubicHermiteSpline.derivative(t, 1)));
      EXPECT_TRUE(cubicSpline.derivative(t, 2).isApprox(cubicHermiteSpline.derivative(t, 2)));
      EXPECT_TRUE(cubicSpline.derivative(t, 3).isApprox(cubicHermiteSpline.derivative(t, 3)));
    }
  }
}

TEST(TestCubicHermiteSpline, CubicHermiteSplineMonotone)
{
  std::map<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>> points = {
      {10, {Eigen::Vector3d(1, 0, 1), Eigen::Vector3d::Zero()}},
      {11, {Eigen::Vector3d(2, 10, 1), Eigen::Vector3d::Zero()}},
      {13, {Eigen::Vector3d(3, -5, 3), Eigen::Vector3d::Zero()}},
      {15, {Eigen::Vector3d(4, -2, 3), Eigen::Vector3d::Zero()}},
      {20, {Eigen::Vector3d(5, -1, 1), Eigen::Vector3d::Zero()}},
      {30, {Eigen::Vector3d(6, 0, 1), Eigen::Vector3d::Zero()}}};

  // setup spline
  BWC::CubicHermiteSpline<Eigen::Vector3d> sp(3, points);
  sp.calcMonotoneVelocity(false, false);
  sp.calcCoeff();

  // check monotonicity
  bool initialIter;
  Eigen::Vector3d prevValue;
  for(auto it = points.begin(); it != std::prev(points.end()); it++)
  {
    initialIter = true;
    Eigen::Vector3d deltaPos = std::next(it)->second.first - it->second.first;

    for(double t = it->first; t < std::next(it)->first; t += 0.01)
    {
      Eigen::Vector3d newValue = sp(t);

      if(initialIter)
      {
        initialIter = false;
      }
      else
      {
        EXPECT_TRUE((deltaPos.cwiseProduct(newValue - prevValue).array() >= 0).all());
        EXPECT_TRUE((deltaPos.cwiseProduct(sp.derivative(t, 1)).array() >= 0).all());
      }

      prevValue = newValue;
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

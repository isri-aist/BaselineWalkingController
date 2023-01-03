/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <BaselineWalkingController/trajectory/CubicInterpolator.h>

TEST(TestCubicInterpolator, Test1)
{
  std::map<double, Eigen::Vector3d> points = {{10, Eigen::Vector3d(0, 10, 123)},
                                              {11, Eigen::Vector3d(1, 100, 123)},
                                              {12, Eigen::Vector3d(2, 0, 123)},
                                              {20, Eigen::Vector3d(3, -10, 123)},
                                              {100, Eigen::Vector3d(4, -100, 123)}};

  // setup interpolator
  BWC::CubicInterpolator<Eigen::Vector3d> interp(points);

  // check for the sampled points of each section
  for(size_t i = 0; i < points.size() - 1; i++)
  {
    double t0 = std::next(points.begin(), i)->first;
    double t1 = std::next(points.begin(), i + 1)->first;
    const Eigen::Vector3d & x0 = std::next(points.begin(), i)->second;
    const Eigen::Vector3d & x1 = std::next(points.begin(), i + 1)->second;
    Eigen::Vector3d x_min = x0.array().min(x1.array());
    Eigen::Vector3d x_max = x0.array().max(x1.array());
    for(double t = t0; t <= t1; t += 0.01)
    {
      Eigen::Vector3d x = interp(t);
      EXPECT_TRUE(((x - x_min).array() > -1e-10).all() && ((x_max - x).array() > -1e-10).all());
    }
  }

  // check for the boundaries of each section
  for(const auto & point : points)
  {
    double t0 = point.first;
    const Eigen::Vector3d & x0 = point.second;
    Eigen::Vector3d x = interp(t0);
    Eigen::Vector3d v = interp.derivative(t0, 1);
    EXPECT_TRUE((x - x0).norm() < 1e-10);
    EXPECT_TRUE(v.norm() < 1e-10);
  }
}

TEST(TestCubicInterpolator, CompareCubicHermiteSplineAndCubicInterpolator)
{
  for(int i = 0; i < 100; i++)
  {
    std::vector<double> t_list = {0.0, 1.0, 2.0, 5.0};
    BWC::CubicHermiteSpline<Eigen::Vector3d> cubicHermiteSpline(3);
    BWC::CubicInterpolator<Eigen::Vector3d> cubicInterp;
    for(size_t j = 0; j < t_list.size(); j++)
    {
      Eigen::Vector3d pos = Eigen::Vector3d::Random();
      cubicHermiteSpline.appendPoint(std::make_pair(t_list[j], std::make_pair(pos, Eigen::Vector3d::Zero())));
      cubicInterp.appendPoint(std::make_pair(t_list[j], pos));
    }
    cubicHermiteSpline.calcCoeff();
    cubicInterp.calcCoeff();

    for(size_t j = 0; j < t_list.size() - 1; j++)
    {
      const int divideNum = 100;
      for(int k = 0; k <= divideNum; k++)
      {
        double ratio = static_cast<double>(k) / divideNum;
        double t = (1.0 - ratio) * t_list[j] + ratio * t_list[j + 1];
        EXPECT_TRUE((cubicHermiteSpline(t) - cubicInterp(t)).norm() < 1e-6) << "t: " << t << std::endl;
        EXPECT_TRUE((cubicHermiteSpline.derivative(t, 1) - cubicInterp.derivative(t, 1)).norm() < 1e-6)
            << "t: " << t << std::endl;
        EXPECT_TRUE((cubicHermiteSpline.derivative(t, 2) - cubicInterp.derivative(t, 2)).norm() < 1e-6)
            << "t: " << t << std::endl;
        EXPECT_TRUE((cubicHermiteSpline.derivative(t, 3) - cubicInterp.derivative(t, 3)).norm() < 1e-6)
            << "t: " << t << std::endl;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

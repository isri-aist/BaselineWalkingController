/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <BaselineWalkingController/trajectory/Func.h>

TEST(TestFunc, PiecewiseFunc)
{
  std::vector<std::pair<double, std::shared_ptr<BWC::Func<Eigen::Vector3d>>>> funcs = {
      {0, std::make_shared<BWC::LinearPolynomial<Eigen::Vector3d>>(
              std::array<Eigen::Vector3d, 2>{Eigen::Vector3d(1, -2, 3), Eigen::Vector3d(-11, 12, -13)})},
      {1, std::make_shared<BWC::QuadraticPolynomial<Eigen::Vector3d>>(std::array<Eigen::Vector3d, 3>{
              Eigen::Vector3d(4, -5, 6), Eigen::Vector3d(-14, 15, -16), Eigen::Vector3d(104, 105, 106)})},
      {5, std::make_shared<BWC::Constant<Eigen::Vector3d>>(Eigen::Vector3d(7, -8, 9))}};

  // setup piecewise function
  BWC::PiecewiseFunc<Eigen::Vector3d> piecewise_func;
  for(const auto & func : funcs)
  {
    piecewise_func.appendFunc(func.first, func.second);
  }

  // check function value
  for(double t = -10; t < 10; t += 0.1)
  {
    for(size_t i = 0; i < funcs.size(); i++)
    {
      if(t <= funcs[i].first)
      {
        EXPECT_TRUE(piecewise_func(t).isApprox((*funcs[i].second)(t)));
        break;
      }
      else if(i == funcs.size() - 1)
      {
        EXPECT_THROW(piecewise_func(t), std::runtime_error);
      }
    }
  }
}

TEST(TestFunc, Polynomial)
{
  std::array<Eigen::Vector3d, 5> coeffs = {Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6), Eigen::Vector3d(7, 8, 9),
                                           Eigen::Vector3d(10, 11, 12), Eigen::Vector3d(13, 14, 15)};

  // setup polynomial function
  BWC::Polynomial<Eigen::Vector3d, 4> poly_func(coeffs);

  // check function value and derivatives
  for(double t = -10; t < 10; t += 0.1)
  {
    EXPECT_TRUE(poly_func(t).isApprox(coeffs[0] + coeffs[1] * t + coeffs[2] * std::pow(t, 2)
                                      + coeffs[3] * std::pow(t, 3) + coeffs[4] * std::pow(t, 4)));

    EXPECT_TRUE(poly_func.derivative(t).isApprox(coeffs[1] + coeffs[2] * 2 * t + coeffs[3] * 3 * std::pow(t, 2)
                                                 + coeffs[4] * 4 * std::pow(t, 3)));

    EXPECT_TRUE(
        poly_func.derivative(t, 2).isApprox(coeffs[2] * 2 + coeffs[3] * 6 * t + coeffs[4] * 12 * std::pow(t, 2)));

    EXPECT_TRUE(poly_func.derivative(t, 3).isApprox(coeffs[3] * 6 + coeffs[4] * 24 * t));

    EXPECT_TRUE(poly_func.derivative(t, 4).isApprox(coeffs[4] * 24));

    EXPECT_TRUE(poly_func.derivative(t, 5).isApprox(Eigen::Vector3d::Zero()));

    EXPECT_TRUE(poly_func.derivative(t, 6).isApprox(Eigen::Vector3d::Zero()));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

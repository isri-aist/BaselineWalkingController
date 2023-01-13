/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <BaselineWalkingController/swing/SwingTrajCubicSplineSimple.h>
#include <BaselineWalkingController/swing/SwingTrajIndHorizontalVertical.h>
#include <BaselineWalkingController/swing/SwingTrajVariableTaskStiffness.h>

template<class SwingTrajType>
void testSwingTraj()
{
  sva::PTransformd startPose = sva::PTransformd(sva::RotZ(-0.1), Eigen::Vector3d(0.1, -0.2, 0.0));
  sva::PTransformd goalPose = sva::PTransformd(sva::RotZ(0.5), Eigen::Vector3d(1.1, 0.2, 0.3));
  double startTime = 1.0;
  double goalTime = 2.5;
  std::shared_ptr<BWC::SwingTraj> swingTraj = std::make_shared<SwingTrajType>(startPose, goalPose, startTime, goalTime);

  const int divideNum = 100;
  for(int i = 0; i <= divideNum; i++)
  {
    double ratio = static_cast<double>(i) / divideNum;
    double t = (1.0 - ratio) * startTime + ratio * goalTime;
    const auto & pose = swingTraj->pose(t);
    const auto & vel = swingTraj->vel(t);
    const auto & accel = swingTraj->accel(t);
    EXPECT_FALSE(pose.translation().array().isNaN().any() || pose.translation().array().isInf().any());
    EXPECT_FALSE(pose.rotation().array().isNaN().any() || pose.rotation().array().isInf().any());
    EXPECT_FALSE(vel.vector().array().isNaN().any() || vel.vector().array().isInf().any());
    EXPECT_FALSE(accel.vector().array().isNaN().any() || accel.vector().array().isInf().any());

    if(i == 0)
    {
      EXPECT_LT(sva::transformError(pose, startPose).vector().norm(), 1e-6);
    }
    else if(i == divideNum)
    {
      EXPECT_LT(sva::transformError(pose, goalPose).vector().norm(), 1e-6);
    }
  }
}

TEST(TestSwingTraj, SwingTrajCubicSplineSimple)
{
  testSwingTraj<BWC::SwingTrajCubicSplineSimple>();
}

TEST(TestSwingTraj, SwingTrajIndHorizontalVertical)
{
  testSwingTraj<BWC::SwingTrajIndHorizontalVertical>();
}

TEST(TestSwingTraj, SwingTrajVariableTaskStiffness)
{
  testSwingTraj<BWC::SwingTrajVariableTaskStiffness>();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

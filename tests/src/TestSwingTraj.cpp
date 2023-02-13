/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <BaselineWalkingController/swing/SwingTrajCubicSplineSimple.h>
#include <BaselineWalkingController/swing/SwingTrajIndHorizontalVertical.h>
#include <BaselineWalkingController/swing/SwingTrajLandingSearch.h>
#include <BaselineWalkingController/swing/SwingTrajVariableTaskGain.h>

template<class SwingTrajType>
void testSwingTraj()
{
  sva::PTransformd startPose = sva::PTransformd(sva::RotZ(-0.1), Eigen::Vector3d(0.1, -0.2, 0.0));
  sva::PTransformd endPose = sva::PTransformd(sva::RotZ(0.5), Eigen::Vector3d(1.1, 0.2, 0.3));
  double startTime = 1.0;
  double endTime = 2.5;
  BWC::TaskGain taskGain = BWC::TaskGain(sva::MotionVecd(Eigen::Vector6d::Constant(100)));
  std::shared_ptr<BWC::SwingTraj> swingTraj =
      std::make_shared<SwingTrajType>(startPose, endPose, startTime, endTime, taskGain);

  const int divideNum = 100;
  for(int i = 0; i <= divideNum; i++)
  {
    double ratio = static_cast<double>(i) / divideNum;
    double t = (1.0 - ratio) * startTime + ratio * endTime;
    const auto & pose = swingTraj->pose(t);
    const auto & vel = swingTraj->vel(t);
    const auto & accel = swingTraj->accel(t);
    const auto & taskGain = swingTraj->taskGain(t);
    const auto & stiffness = taskGain.stiffness;
    const auto & damping = taskGain.damping;
    EXPECT_FALSE(pose.translation().array().isNaN().any() || pose.translation().array().isInf().any());
    EXPECT_FALSE(pose.rotation().array().isNaN().any() || pose.rotation().array().isInf().any());
    EXPECT_FALSE(vel.vector().array().isNaN().any() || vel.vector().array().isInf().any());
    EXPECT_FALSE(accel.vector().array().isNaN().any() || accel.vector().array().isInf().any());
    EXPECT_FALSE(stiffness.vector().array().isNaN().any() || stiffness.vector().array().isInf().any());
    EXPECT_FALSE(damping.vector().array().isNaN().any() || damping.vector().array().isInf().any());

    if(i == 0)
    {
      EXPECT_LT(sva::transformError(pose, startPose).vector().norm(), 1e-6);
    }
    else if(i == divideNum)
    {
      EXPECT_LT(sva::transformError(pose, endPose).vector().norm(), 1e-6);
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

TEST(TestSwingTraj, SwingTrajVariableTaskGain)
{
  testSwingTraj<BWC::SwingTrajVariableTaskGain>();
}

TEST(TestSwingTraj, SwingTrajLandingSearch)
{
  testSwingTraj<BWC::SwingTrajLandingSearch>();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

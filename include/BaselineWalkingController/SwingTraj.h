#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace BWC
{
/** \brief Foot swing trajectory. */
class SwingTraj
{
public:
  /** \brief Constructor.
      \param startPose start pose
      \param goalPose pose goal pose
      \param startTime start time
      \param goalTime goal time
  */
  SwingTraj(const sva::PTransformd & startPose, const sva::PTransformd & goalPose, double startTime, double goalTime)
  : startPose_(startPose), goalPose_(goalPose), startTime_(startTime), goalTime_(goalTime)
  {
  }

  /** \brief Calculate the pose of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::PTransformd pose(double t) const = 0;

  /** \brief Calculate the velocity of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd vel(double t) const = 0;

  /** \brief Calculate the acceleration of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd accel(double t) const = 0;

public:
  //! Start pose
  sva::PTransformd startPose_ = sva::PTransformd::Identity();

  //! Goal pose
  sva::PTransformd goalPose_ = sva::PTransformd::Identity();

  //! Start time [sec]
  double startTime_ = 0.0;

  //! Goal time [sec]
  double goalTime_ = 0.0;
};
} // namespace BWC

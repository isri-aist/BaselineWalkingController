#pragma once

#include <BaselineWalkingController/SwingTraj.h>

namespace BWC
{
template<class T>
class PiecewiseFunc;
template<class T, class U>
class CubicInterpolator;

/** \brief Simple foot swing trajectory with cubic spline.

    The position is interpolated by a single 3D cubic spline. The rotation is interpolated by a single cubic
   interpolator.
 */
class SwingTrajCubicSplineSimple : public SwingTraj
{
public:
  /** \brief Configuration. */
  struct Configuration : public SwingTraj::Configuration
  {
    //! Duration ratio to withdraw foot
    double withdrawDurationRatio = 0.25;

    //! Position offset to withdraw foot [m]
    Eigen::Vector3d withdrawOffset = Eigen::Vector3d(0, 0, 0.015);

    //! Duration ratio to approach foot
    double approachDurationRatio = 0.25;

    //! Position offset to approach foot [m]
    Eigen::Vector3d approachOffset = Eigen::Vector3d(0, 0, 0.015);

    //! Position offset to swing foot [m]
    Eigen::Vector3d swingOffset = Eigen::Vector3d(0, 0, 0.05);

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

public:
  /** \brief Constructor.
      \param startPose start pose
      \param goalPose pose goal pose
      \param startTime start time
      \param goalTime goal time
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTrajCubicSplineSimple(const sva::PTransformd & startPose,
                             const sva::PTransformd & goalPose,
                             double startTime,
                             double goalTime,
                             const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Calculate the pose of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::PTransformd pose(double t) const override;

  /** \brief Calculate the velocity of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd vel(double t) const override;

  /** \brief Calculate the acceleration of the swing trajectory at a specified time.
      \param t time
  */
  virtual sva::MotionVecd accel(double t) const override;

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

protected:
  //! Configuration
  Configuration config_;

  //! Position function
  std::shared_ptr<PiecewiseFunc<Eigen::Vector3d>> posFunc_;

  //! Rotation function
  std::shared_ptr<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> rotFunc_;
};
} // namespace BWC

#pragma once

#include <BaselineWalkingController/SwingTraj.h>
#include <BaselineWalkingController/trajectory/CubicInterpolator.h>
#include <BaselineWalkingController/trajectory/CubicSpline.h>

namespace BWC
{
/** \brief Simple foot swing trajectory with cubic spline.

    The horizontal position is interpolated by a cubic interpolator. The vertical position is interpolated by a cubic
   spline. The rotation is interpolated by a cubic interpolator.
 */
class SwingTrajIndHorizontalVertical : public SwingTraj
{
protected:
  //! 1D vector
  using Vector1d = Eigen::Matrix<double, 1, 1>;

public:
  /** \brief Configuration. */
  struct Configuration : public SwingTraj::Configuration
  {
    //! Duration ratio to withdraw foot
    double withdrawDurationRatio = 0.25;

    //! Duration ratio to approach foot
    double approachDurationRatio = 0.25;

    //! Duration ratio of vertical top
    double verticalTopDurationRatio = 0.5;

    //! Position offset of vertical top [m]
    Eigen::Vector3d verticalTopOffset = Eigen::Vector3d(0, 0, 0.05);

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
  SwingTrajIndHorizontalVertical(const sva::PTransformd & startPose,
                                 const sva::PTransformd & goalPose,
                                 double startTime,
                                 double goalTime,
                                 const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Get type of foot swing trajectory. */
  inline virtual std::string type() const override
  {
    return "IndHorizontalVertical";
  }

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

  //! Horizontal position function
  std::shared_ptr<CubicInterpolator<Eigen::Vector2d>> horizontalPosFunc_;

  //! Vertical position function
  std::shared_ptr<CubicSpline<Vector1d>> verticalPosFunc_;

  //! Rotation function
  std::shared_ptr<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> rotFunc_;
};
} // namespace BWC

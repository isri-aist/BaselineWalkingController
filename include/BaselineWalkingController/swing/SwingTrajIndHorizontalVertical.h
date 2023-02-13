#pragma once

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>

#include <TrajColl/CubicInterpolator.h>
#include <TrajColl/CubicSpline.h>

#include <BaselineWalkingController/SwingTraj.h>

namespace BWC
{
/** \brief Foot swing trajectory with independent interpolation of horizontal and vertical positions

    The horizontal position is interpolated by a cubic interpolator. The vertical position is interpolated by a cubic
   spline. The rotation is interpolated by a cubic interpolator. The tilt angle of the foot during withdraw and approach
   can be specified.
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

    //! Tilt angle in withdraw [rad]
    double tiltAngleWithdraw = mc_rtc::constants::toRad(20);

    //! Tilt angle in approach [rad]
    double tiltAngleApproach = mc_rtc::constants::toRad(10);

    //! Duration ratio to set tilt angle in withdraw
    double tiltAngleWithdrawDurationRatio = 0.25;

    //! Duration ratio to set tilt angle in approach
    double tiltAngleApproachDurationRatio = 0.25;

    //! Duration ratio at which the tilt center starts to change
    double tiltCenterWithdrawDurationRatio = 0.25;

    //! Duration ratio at which the tilt center ends to change
    double tiltCenterApproachDurationRatio = 0.25;

    //! Threshold distance between start pose and end pose to enable tilt [m]
    double tiltDistThre = 0.2;

    //! Threshold of forward angle between start pose and end pose to enable tilt [rad]
    double tiltForwardAngleThre = mc_rtc::constants::toRad(10);

    /** \brief Constructor.

        This is necessary for https://stackoverflow.com/q/53408962
    */
    Configuration() {}

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

public:
  //! Default configuration
  static inline Configuration defaultConfig_;

  /** \brief Load mc_rtc configuration to the default configuration.
      \param mcRtcConfig mc_rtc configuration
  */
  static void loadDefaultConfig(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Add entries of default configuration to the GUI.
      \param gui GUI
      \param category category of GUI entries
   */
  static void addConfigToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

  /** \brief Remove entries of default configuration from the GUI.
      \param gui GUI
      \param category category of GUI entries
   */
  static void removeConfigFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

public:
  /** \brief Constructor.
      \param startPose start pose
      \param endPose pose end pose
      \param startTime start time
      \param endTime end time
      \param taskGain IK task gain
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTrajIndHorizontalVertical(const sva::PTransformd & startPose,
                                 const sva::PTransformd & endPose,
                                 double startTime,
                                 double endTime,
                                 const TaskGain & taskGain,
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
  /** \brief Accessor to the configuration. */
  inline virtual Configuration & config() override
  {
    return config_;
  }

protected:
  //! Configuration
  Configuration config_ = defaultConfig_;

  //! Horizontal position function
  std::shared_ptr<TrajColl::CubicInterpolator<Eigen::Vector2d>> horizontalPosFunc_;

  //! Vertical position function
  std::shared_ptr<TrajColl::CubicSpline<Vector1d>> verticalPosFunc_;

  //! Rotation function
  std::shared_ptr<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> rotFunc_;

  //! Tilt angle function
  std::shared_ptr<TrajColl::CubicInterpolator<Vector1d>> tiltAngleFunc_;

  //! Tilt center function
  std::shared_ptr<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>> tiltCenterFunc_;
};
} // namespace BWC

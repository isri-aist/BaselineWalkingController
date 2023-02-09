#pragma once

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>

#include <TrajColl/CubicInterpolator.h>
#include <TrajColl/CubicSpline.h>

#include <BaselineWalkingController/SwingTraj.h>

namespace BWC
{
/** \brief Foot swing trajectory that makes the horizontal position converge by determining the variable IK task gain.

    IK task gain (stiffness and damping) of horizontal position are determined by optimal control. The vertical
   position is interpolated by a cubic spline. The rotation is interpolated by a cubic interpolator.
 */
class SwingTrajVariableTaskGain : public SwingTraj
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
  SwingTrajVariableTaskGain(const sva::PTransformd & startPose,
                            const sva::PTransformd & endPose,
                            double startTime,
                            double endTime,
                            const TaskGain & taskGain,
                            const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Get type of foot swing trajectory. */
  inline virtual std::string type() const override
  {
    return "VariableTaskGain";
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

  /** \brief Calculate the IK task gain of the swing trajectory at a specified time.
      \param t time
  */
  virtual TaskGain taskGain(double t) const override;

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

public:
  //! Withdraw time
  double withdrawTime_ = 0;

  //! Approach time
  double approachTime_ = 0;

protected:
  //! Configuration
  Configuration config_ = defaultConfig_;

  //! Vertical position function
  std::shared_ptr<TrajColl::CubicSpline<Vector1d>> verticalPosFunc_;
};
} // namespace BWC

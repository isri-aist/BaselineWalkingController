#pragma once

#include <mc_rtc/gui/StateBuilder.h>

#include <TrajColl/CubicInterpolator.h>

#include <BaselineWalkingController/SwingTraj.h>

namespace BWC
{
/** \brief Foot swing trajectory, moving the foot while searching for the landing pose on the ground.

    The position and orientation is interpolated by a single cubic interpolator.
 */
class SwingTrajLandingSearch : public SwingTraj
{
public:
  /** \brief Configuration. */
  struct Configuration : public SwingTraj::Configuration
  {
    //! Duration ratio to withdraw foot
    double withdrawDurationRatio = 0.2;

    //! Position offset to withdraw foot [m]
    Eigen::Vector3d withdrawOffset = Eigen::Vector3d(0, 0, 0.04);

    //! Duration ratio to pre-approach foot
    double preApproachDurationRatio = 0.25;

    //! Duration ratio to approach foot
    double approachDurationRatio = 0.2;

    //! Position offset to approach foot [m]
    Eigen::Vector3d approachOffset = Eigen::Vector3d(0, 0, 0.04);

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
  SwingTrajLandingSearch(const sva::PTransformd & startPose,
                         const sva::PTransformd & endPose,
                         double startTime,
                         double endTime,
                         const TaskGain & taskGain,
                         const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Get type of foot swing trajectory. */
  inline virtual std::string type() const override
  {
    return "LandingSearch";
  }

  /** \brief Update the internal state of the swing trajectory.
      \param t time
  */
  virtual void update(double t) override;

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

  //! Waypoint pose list
  std::map<double, sva::PTransformd> waypointPoseList_;

  //! Pose function
  std::shared_ptr<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>> poseFunc_;

  //! Whether the end pose has been finalized
  bool finalizeEndPose_ = false;
};
} // namespace BWC

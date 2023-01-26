#pragma once

#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <BaselineWalkingController/RobotUtils.h>

namespace BWC
{
/** \brief Foot swing trajectory. */
class SwingTraj
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    virtual void load(const mc_rtc::Configuration & // mcRtcConfig
    )
    {
    }
  };

public:
  /** \brief Constructor.
      \param startPose start pose
      \param endPose pose end pose
      \param startTime start time
      \param endTime end time
      \param taskGain IK task gain
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTraj(const sva::PTransformd & startPose,
            const sva::PTransformd & endPose,
            double startTime,
            double endTime,
            const TaskGain & taskGain,
            const mc_rtc::Configuration & mcRtcConfig = {})
  : startPose_(startPose), endPose_(endPose), startTime_(startTime), endTime_(endTime), taskGain_(taskGain)
  {
    config_.load(mcRtcConfig);
  }

  /** \brief Get type of foot swing trajectory. */
  virtual std::string type() const = 0;

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

  /** \brief Calculate the IK task gain of the swing trajectory at a specified time.
      \param t time
  */
  inline virtual TaskGain taskGain(double // t
  ) const
  {
    return taskGain_;
  }

  /** \brief Notify touch down detection.
      \param t time
  */
  inline virtual void touchDown(double t)
  {
    touchDownTime_ = t;
  }

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const
  {
    return config_;
  }

public:
  //! Configuration
  Configuration config_;

  //! Start pose
  sva::PTransformd startPose_ = sva::PTransformd::Identity();

  //! End pose
  sva::PTransformd endPose_ = sva::PTransformd::Identity();

  //! Start time [sec]
  double startTime_ = 0.0;

  //! End time [sec]
  double endTime_ = 0.0;

  //! IK task gain
  TaskGain taskGain_;

protected:
  //! Time when touch down is detected (-1 if not detected)
  double touchDownTime_ = -1;
};
} // namespace BWC

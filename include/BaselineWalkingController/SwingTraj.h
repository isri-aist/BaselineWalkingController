#pragma once

#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>

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
      \param goalPose pose goal pose
      \param startTime start time
      \param goalTime goal time
      \param mcRtcConfig mc_rtc configuration
  */
  SwingTraj(const sva::PTransformd & startPose,
            const sva::PTransformd & goalPose,
            double startTime,
            double goalTime,
            const mc_rtc::Configuration & mcRtcConfig = {})
  : startPose_(startPose), goalPose_(goalPose), startTime_(startTime), goalTime_(goalTime)
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

  //! Goal pose
  sva::PTransformd goalPose_ = sva::PTransformd::Identity();

  //! Start time [sec]
  double startTime_ = 0.0;

  //! Goal time [sec]
  double goalTime_ = 0.0;
};
} // namespace BWC

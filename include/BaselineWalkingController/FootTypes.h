#pragma once

#include <mc_rtc/Configuration.h>

namespace BWC
{
/** \brief Foot. */
enum class Foot
{
  //! Left foot
  Left = 0,

  //! Right foot
  Right
};

namespace Feet
{
//! Both feet
const std::set<Foot> Both = {Foot::Left, Foot::Right};
} // namespace Feet

/** \brief Convert string to foot. */
Foot strToFoot(const std::string & footStr);

/** \brief Get the opposite foot. */
Foot opposite(const Foot & foot);

/** \brief Get the sign of foot.

    Positive for left foot, negative for right foot.
*/
int sign(const Foot & foot);

/** \brief Support phase. */
enum class SupportPhase
{
  DoubleSupport = 0,
  LeftSupport,
  RightSupport
};

/** \brief Footstep. */
struct Footstep
{
  /** \brief Configuration. */
  struct Configuration
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
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

  /** \brief Constructor.
      \param _foot foot
      \param _pose foot pose
      \param _transitStartTime time to start ZMP transition
      \param _swingStartTime time to start swinging the foot
      \param _swingEndTime time to end swinging the foot
      \param _transitEndTime time to end ZMP transition

      \note The following relation must hold: _transitStartTime < _swingStartTime < _swingEndTime < _transitEndTime.
  */
  Footstep(Foot _foot,
           sva::PTransformd _pose,
           double _transitStartTime = 0,
           double _swingStartTime = 0,
           double _swingEndTime = 0,
           double _transitEndTime = 0)
  : foot(_foot), pose(_pose), transitStartTime(_transitStartTime), swingStartTime(_swingStartTime),
    swingEndTime(_swingEndTime), transitEndTime(_transitEndTime){};

  //! Foot
  Foot foot;

  //! Foot pose
  sva::PTransformd pose;

  //! Time to start ZMP transition
  double transitStartTime;

  //! Time to start swinging the foot
  double swingStartTime;

  //! Time to end swinging the foot
  double swingEndTime;

  //! Time to end ZMP transition
  double transitEndTime;

  //! Configuration
  Configuration config;
};
} // namespace BWC

namespace std
{
/** \brief Convert foot to string. */
std::string to_string(const BWC::Foot & foot);

/** \brief Convert support phase to string. */
std::string to_string(const BWC::SupportPhase & phase);
} // namespace std

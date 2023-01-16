#pragma once

#include <set>

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
  //! Both feet support phase
  DoubleSupport = 0,

  //! Left foot support phase
  LeftSupport,

  //! Right foot support phase
  RightSupport
};

/** \brief Footstep. */
struct Footstep
{
  /** \brief Constructor.
      \param _foot foot
      \param _pose foot pose
      \param _transitStartTime time to start ZMP transition
      \param _swingStartTime time to start swinging the foot
      \param _swingEndTime time to end swinging the foot
      \param _transitEndTime time to end ZMP transition
      \param _swingTrajConfig configuration for swing trajectory

      \note The following relation must hold: _transitStartTime < _swingStartTime < _swingEndTime < _transitEndTime.
  */
  Footstep(Foot _foot,
           const sva::PTransformd & _pose,
           double _transitStartTime = 0,
           double _swingStartTime = 0,
           double _swingEndTime = 0,
           double _transitEndTime = 0,
           const mc_rtc::Configuration & _swingTrajConfig = {})
  : foot(_foot), pose(_pose), transitStartTime(_transitStartTime), swingStartTime(_swingStartTime),
    swingEndTime(_swingEndTime), transitEndTime(_transitEndTime), swingTrajConfig(_swingTrajConfig)
  {
  }

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

  //! Configuration for swing trajectory
  mc_rtc::Configuration swingTrajConfig = {};
};
} // namespace BWC

namespace std
{
/** \brief Convert foot to string. */
std::string to_string(const BWC::Foot & foot);

/** \brief Convert support phase to string. */
std::string to_string(const BWC::SupportPhase & phase);
} // namespace std

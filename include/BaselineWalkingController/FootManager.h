#pragma once

#include <deque>
#include <unordered_map>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/ImpedanceGains.h>

#include <BaselineWalkingController/FootTypes.h>
#include <BaselineWalkingController/trajectory/CubicInterpolator.h>
#include <BaselineWalkingController/trajectory/Func.h>

namespace BWC
{
class BaselineWalkingController;
class Contact;

/** Foot manager.

    Foot manager generates a swing foot trajectory and ZMP trajectory from a specified footstep sequence.
*/
class FootManager
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "FootManager";

    //! Duration of one footstep. [sec]
    double footstepDuration = 1.0;

    //! Duration ratio of double support phase
    double doubleSupportRatio = 0.2;

    //! Transformation from foot midpose to each foot pose
    std::unordered_map<Foot, sva::PTransformd> midToFootTranss = {
        {Foot::Left, sva::PTransformd(Eigen::Vector3d(0, 0.1, 0))},
        {Foot::Right, sva::PTransformd(Eigen::Vector3d(0, -0.1, 0))}};

    //! Horizon of ZMP trajectory [sec]
    double zmpHorizon = 2.0;

    //! ZMP offset of each foot (positive for x-forward, y-outside, z-upward) [m]
    Eigen::Vector3d zmpOffset = Eigen::Vector3d::Zero();

    //! Whether to overwrite landing pose so that the relative pose from support foot to swing foot is retained
    bool overwriteLandingPose = false;

    //! Whether to stop swing trajectory for touch down foot
    bool stopSwingTrajForTouchDownFoot = true;

    //! Whether to keep foot pose of touch down foot during support phase
    bool keepSupportFootPoseForTouchDownFoot = false;

    //! Whether to enable wrench distribution for touch down foot
    bool enableWrenchDistForTouchDownFoot = true;

    //! Friction coefficient of foot contact
    double fricCoeff = 0.5;

    //! Thresholds for touch down detection
    //! @{
    double touchDownRemainingDuration = 0.2; // [sec]
    double touchDownPosError = 0.05; // [m]
    double touchDownForceZ = 50; // [N]
    //! @}

    //! Impedance gains for foot tasks
    std::unordered_map<std::string, mc_tasks::force::ImpedanceGains> impGains = {
        {"singleSupport", mc_tasks::force::ImpedanceGains::Default()},
        {"doubleSupport", mc_tasks::force::ImpedanceGains::Default()},
        {"swing", mc_tasks::force::ImpedanceGains::Default()}};

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

protected:
  //! 1D vector
  using Vector1d = Eigen::Matrix<double, 1, 1>;

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
  */
  FootManager(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
  */
  void reset();

  /** \brief Update.

      This method should be called once every control cycle.
  */
  void update();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Add entries to the GUI. */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Get foot surface name. */
  const std::string & surfaceName(const Foot & foot) const;

  /** \brief Make a footstep.
      \param foot foot
      \param footMidpose middle pose of both feet
      \param startTime time to start the footstep
      \param mcRtcConfig mc_rtc configuration
  */
  Footstep makeFootstep(const Foot & foot,
                        const sva::PTransformd & footMidpose,
                        double startTime,
                        const mc_rtc::Configuration & mcRtcConfig = {}) const;

  /** \brief Append a target footstep to the queue.
      \param newFootstep footstep to append
      \return whether newFootstep is appended
  */
  bool appendFootstep(const Footstep & newFootstep);

  /** \brief Calculate reference ZMP.
      \param t time
      \param derivOrder derivative order (0 for original value, 1 for velocity)
  */
  Eigen::Vector3d calcRefZmp(double t, int derivOrder = 0) const;

  /** \brief Calculate reference ground Z position.
      \param t time
      \param derivOrder derivative order (0 for original value, 1 for velocity)
  */
  double calcRefGroundPosZ(double t, int derivOrder = 0) const;

  /** \brief Calculate contact foot poses.
      \param t time

      Touch down foot is NOT included.

      \see FootManager::calcCurrentContactList
  */
  std::unordered_map<Foot, sva::PTransformd> calcContactFootPoses(double t) const;

  /** \brief Get current contact feet.

      If FootManager::Configuration::enableWrenchDistForTouchDownFoot is true, the touch down foot is also included.
  */
  std::set<Foot> getCurrentContactFeet() const;

  /** \brief Calculate current contact list.

      If FootManager::Configuration::enableWrenchDistForTouchDownFoot is true, the touch down foot is also included.

      \see FootManager::calcContactFootPoses
  */
  std::unordered_map<Foot, std::shared_ptr<Contact>> calcCurrentContactList() const;

  /** \brief Get the support ratio of left foot.

      1 for full left foot support, 0 for full right foot support.
  */
  double leftFootSupportRatio() const;

  /** \brief Calculate ZMP with offset.
      \param foot foot
      \param footPose foot pose
  */
  Eigen::Vector3d calcZmpWithOffset(const Foot & foot, const sva::PTransformd & footPose) const;

  /** \brief Calculate ZMP with offset.
      \param footPoses foot poses

      Returns zero if footPoses is empty
  */
  Eigen::Vector3d calcZmpWithOffset(const std::unordered_map<Foot, sva::PTransformd> & footPoses) const;

  /** \brief Access footstep queue. */
  inline const std::deque<Footstep> & footstepQueue() const noexcept
  {
    return footstepQueue_;
  }

  /** \brief Access footstep queue.

      \note Changing footstep queue directly is dangerous and should be avoided if possible. To safely add a footstep to
      footstep queue, call appendFootstep().
  */
  inline std::deque<Footstep> & footstepQueue() noexcept
  {
    return footstepQueue_;
  }

  /** \brief Access previous footstep. */
  std::shared_ptr<Footstep> prevFootstep() const
  {
    return prevFootstep_;
  }

  /** \brief Get the target foot pose represented in world frame. */
  inline const sva::PTransformd & targetFootPose(const Foot & foot) const
  {
    return targetFootPoses_.at(foot);
  }

  /** \brief Get the target foot velocity represented in world frame. */
  inline const sva::MotionVecd & targetFootVel(const Foot & foot) const
  {
    return targetFootVels_.at(foot);
  }

  /** \brief Get the target foot acceleration represented in world frame. */
  inline const sva::MotionVecd & targetFootAccel(const Foot & foot) const
  {
    return targetFootAccels_.at(foot);
  }

  /** \brief Get the support phase. */
  inline const SupportPhase supportPhase() const noexcept
  {
    return supportPhase_;
  }

protected:
  /** \brief Const accessor to the controller. */
  inline const BaselineWalkingController & ctl() const
  {
    return *ctlPtr_;
  }

  /** \brief Accessor to the controller. */
  inline BaselineWalkingController & ctl()
  {
    return *ctlPtr_;
  }

  /** \brief Update foot tasks. */
  void updateFootTraj();

  /** \brief Update ZMP trajectory. */
  void updateZmpTraj();

  /** \brief Get the remaining duration for next touch down.

      Returns zero in double support phase. */
  double touchDownRemainingDuration() const;

  /** \brief Detect touch down.
      \return true if touch down is detected during swing
  */
  bool detectTouchDown() const;

protected:
  //! Configuration
  Configuration config_;

  //! Pointer to controller
  BaselineWalkingController * ctlPtr_ = nullptr;

  //! Footstep queue
  std::deque<Footstep> footstepQueue_;

  //! Previous footstep
  std::shared_ptr<Footstep> prevFootstep_;

  //! Target foot pose represented in world frame
  std::unordered_map<Foot, sva::PTransformd> targetFootPoses_;

  //! Target foot velocity represented in world frame
  std::unordered_map<Foot, sva::MotionVecd> targetFootVels_;

  //! Target foot acceleration represented in world frame
  std::unordered_map<Foot, sva::MotionVecd> targetFootAccels_;

  //! Foot poses in the last double support phase
  std::unordered_map<Foot, sva::PTransformd> lastDoubleSupportFootPoses_;

  //! Support phase
  SupportPhase supportPhase_ = SupportPhase::DoubleSupport;

  //! ZMP function
  std::shared_ptr<CubicInterpolator<Eigen::Vector3d>> zmpFunc_;

  //! Ground Z position function
  std::shared_ptr<CubicInterpolator<Vector1d>> groundPosZFunc_;

  //! Contact foot poses list
  std::map<double, std::unordered_map<Foot, sva::PTransformd>> contactFootPosesList_;

  //! Footstep during swing
  const Footstep * swingFootstep_ = nullptr;

  //! Swing foot trajectory
  //! @{
  std::shared_ptr<PiecewiseFunc<Eigen::Vector3d>> swingPosFunc_;
  std::shared_ptr<CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> swingRotFunc_;
  //! @}

  //! Base link Yaw trajectory
  std::shared_ptr<CubicInterpolator<Vector1d>> baseYawFunc_;

  //! Whether touch down is detected during swing
  bool touchDown_ = false;

  //! Types of impedance gains
  std::unordered_map<Foot, std::string> impGainTypes_;

  //! Whether to require updating impedance gains
  bool requireImpGainUpdate_ = true;
};
} // namespace BWC

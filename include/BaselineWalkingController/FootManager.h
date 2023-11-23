#pragma once

#include <deque>
#include <unordered_map>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/ImpedanceGains.h>

#include <TrajColl/CubicInterpolator.h>
#include <TrajColl/CubicSpline.h>

#include <BaselineWalkingController/FootTypes.h>
#include <BaselineWalkingController/RobotUtils.h>

namespace ForceColl
{
class Contact;
}

namespace BWC
{
class BaselineWalkingController;
class SwingTraj;

/** \brief Foot manager.

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

    //! Limit of foot midpose transformation for one footstep (x [m], y [m], theta [rad])
    Eigen::Vector3d deltaTransLimit = Eigen::Vector3d(0.15, 0.1, mc_rtc::constants::toRad(15));

    //! Transformation from foot midpose to each foot pose
    std::unordered_map<Foot, sva::PTransformd> midToFootTranss = {
        {Foot::Left, sva::PTransformd(Eigen::Vector3d(0, 0.1, 0))},
        {Foot::Right, sva::PTransformd(Eigen::Vector3d(0, -0.1, 0))}};

    //! Foot task gains
    TaskGain footTaskGain = TaskGain(sva::MotionVecd(Eigen::Vector6d::Constant(1000)));

    //! Horizon of ZMP trajectory [sec]
    double zmpHorizon = 2.0;

    //! ZMP offset of each foot (positive for x-forward, y-outside, z-upward) [m]
    Eigen::Vector3d zmpOffset = Eigen::Vector3d::Zero();

    //! Default swing trajectory type
    std::string defaultSwingTrajType = "IndHorizontalVertical";

    //! Whether to overwrite landing pose so that the relative pose from support foot to swing foot is retained
    bool overwriteLandingPose = false;

    //! Whether to stop swing trajectory for touch down foot
    bool stopSwingTrajForTouchDownFoot = true;

    //! Whether to keep foot pose of touch down foot during support phase
    bool keepPoseForTouchDownFoot = false;

    //! Whether to enable wrench distribution for touch down foot
    bool enableWrenchDistForTouchDownFoot = true;

    //! Whether to enable arm swing
    bool enableArmSwing = false;

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
        {"SingleSupport", mc_tasks::force::ImpedanceGains::Default()},
        {"DoubleSupport", mc_tasks::force::ImpedanceGains::Default()},
        {"Swing", mc_tasks::force::ImpedanceGains::Default()}};

    //! Arm swing joint angles
    std::unordered_map<std::string, std::map<std::string, std::vector<double>>> jointAnglesForArmSwing = {
        {"Nominal", {}},
        {"Left", {}},
        {"Right", {}}};

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

  /** \brief Velocity mode data.

      In the velocity mode, the robot walks at the specified velocity.
  */
  class VelModeData
  {
  public:
    /** \brief Configuration. */
    struct Configuration
    {
      //! Queue size of footsteps to be sent in the velocity mode (must be at least 3)
      int footstepQueueSize = 3;

      //! Whether to enable online footstep update during swing in the velocity mode
      bool enableOnlineFootstepUpdate = true;

      /** \brief Load mc_rtc configuration.
          \param mcRtcConfig mc_rtc configuration
      */
      void load(const mc_rtc::Configuration & mcRtcConfig);
    };

  public:
    /** \brief Constructor. */
    VelModeData() {}

    /** \brief Reset.
        \param enabled whether the velocity mode is enabled
     */
    void reset(bool enabled);

  public:
    //! Configuration
    Configuration config_;

    //! Whether the velocity mode is enabled
    bool enabled_ = false;

    //! Relative target velocity of foot midpose in the velocity mode (x [m/s], y [m/s], theta [rad/s])
    Eigen::Vector3d targetVel_ = Eigen::Vector3d::Zero();
  };

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
  virtual void update();

  /** \brief Stop.

      This method should be called once when stopping the controller.
  */
  void stop();

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

  /** \brief Const accessor to the velocity mode data. */
  inline const VelModeData & velModeData() const noexcept
  {
    return velModeData_;
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
      \param swingTrajConfig configuration for swing trajectory
  */
  Footstep makeFootstep(const Foot & foot,
                        const sva::PTransformd & footMidpose,
                        double startTime,
                        const mc_rtc::Configuration & swingTrajConfig = {}) const;

  /** \brief Append a target footstep to the queue.
      \param newFootstep footstep to append
      \return whether newFootstep is appended
  */
  bool appendFootstep(const Footstep & newFootstep);

  /** \brief Clear footstep queue (with retaining footstep during swing). */
  void clearFootstepQueue();

  /** \brief Calculate reference ZMP.
      \param t time
      \param derivOrder derivative order (0 for original value, 1 for velocity)
  */
  Eigen::Vector3d calcRefZmp(double t, int derivOrder = 0) const;

  /** \brief Clamp a foot midpose transformation with limit
      \param deltaTrans foot midpose transformation
      \param foot foot
  */
  Eigen::Vector3d clampDeltaTrans(const Eigen::Vector3d & deltaTrans, const Foot & foot);

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
  std::unordered_map<Foot, std::shared_ptr<ForceColl::Contact>> calcCurrentContactList() const;

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
  inline SupportPhase supportPhase() const noexcept
  {
    return supportPhase_;
  }

  /** \brief Send footstep sequence to walk to the relative target pose.
      \param targetTrans relative target pose of foot midpose (x [m], y [m], theta [rad])
      \param lastFootstepNum number of last footstep
      \param waypointTransList waypoint pose list of foot midpose relative to current pose (x [m], y [m], theta [rad])
      \return whether footstep is successfully sent
   */
  bool walkToRelativePose(const Eigen::Vector3d & targetTrans,
                          int lastFootstepNum = 0,
                          const std::vector<Eigen::Vector3d> & waypointTransList = {});

  /** \brief Start velocity mode.
      \return whether it is successfully started
   */
  bool startVelMode();

  /** \brief End velocity mode.
      \return whether it is successfully ended
   */
  bool endVelMode();

  /** \brief Set the relative target velocity
      \param targetVel relative target velocity of foot midpose in the velocity mode (x [m/s], y [m/s], theta [rad/s])
   */
  inline void setRelativeVel(const Eigen::Vector3d & targetVel)
  {
    velModeData_.targetVel_ = targetVel;
  }

  /** \brief Whether the velocity mode is enabled. */
  inline bool velModeEnabled() const
  {
    return velModeData_.enabled_;
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
  virtual void updateFootTraj();

  /** \brief Update ZMP trajectory. */
  virtual void updateZmpTraj();

  /** \brief Update footstep sequence for the velocity mode. */
  void updateVelMode();

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

  //! Velocity mode data
  VelModeData velModeData_;

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

  //! Foot task gains
  std::unordered_map<Foot, TaskGain> footTaskGains_;

  //! Foot poses of start of trajectory
  std::unordered_map<Foot, sva::PTransformd> trajStartFootPoses_;

  //! Functions for foot poses of start of trajectory
  std::unordered_map<Foot, std::shared_ptr<TrajColl::CubicInterpolator<sva::PTransformd, sva::MotionVecd>>>
      trajStartFootPoseFuncs_;

  //! Support phase
  SupportPhase supportPhase_ = SupportPhase::DoubleSupport;

  //! ZMP function
  std::shared_ptr<TrajColl::CubicInterpolator<Eigen::Vector3d>> zmpFunc_;

  //! Ground Z position function
  std::shared_ptr<TrajColl::CubicInterpolator<double>> groundPosZFunc_;

  //! Map of start time and contact foot poses
  std::map<double, std::unordered_map<Foot, sva::PTransformd>> contactFootPosesList_;

  //! Footstep during swing
  Footstep * swingFootstep_ = nullptr;

  //! Foot swing trajectory
  std::shared_ptr<SwingTraj> swingTraj_ = nullptr;

  //! Base link Yaw trajectory
  std::shared_ptr<TrajColl::CubicInterpolator<Eigen::Matrix3d, Eigen::Vector3d>> baseYawFunc_;

  //! Arm swing joint angles trajectory
  std::shared_ptr<TrajColl::CubicSpline<Eigen::VectorXd>> armSwingFunc_;

  //! Whether touch down is detected during swing
  bool touchDown_ = false;

  //! Types of impedance gains for foot tasks
  std::unordered_map<Foot, std::string> impGainTypes_;

  //! Whether to require updating impedance gains for foot tasks
  bool requireImpGainUpdate_ = true;
};
} // namespace BWC

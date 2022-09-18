#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <BaselineWalkingController/FootTypes.h>

namespace mc_rbdyn
{
class Robot;
}

namespace BWC
{
class BaselineWalkingController;
class Contact;
class WrenchDistribution;

/** \brief Centroidal manager.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
 */
class CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Name
    std::string name = "CentroidalManager";

    //! Method
    std::string method = "";

    //! Whether to use actual state for MPC
    bool useActualStateForMpc = false;

    //! Whether to enable DCM feedback
    bool enableZmpFeedback = true;

    //! Whether to enable CoM Z feedback
    bool enableComZFeedback = true;

    /** \brief Feedback gain of DCM

        It must be greater than 1 to be stable.
    */
    double dcmGainP = 2.0;

    //! Feedforward gain of ZMP velocity
    double zmpVelGain = 0.1;

    //! Feedback proportional gain of CoM Z
    double comZGainP = 100.0;

    //! Feedback derivative gain of CoM Z
    double comZGainD = 10.0;

    //! Reference CoM Z position [m]
    double refComZ = 0.9;

    //! Whether to use target surface pose for anchor frame of control robot
    bool useTargetPoseForControlRobotAnchorFrame = true;

    //! Whether to use actual CoM for wrench distribution
    bool useActualComForWrenchDist = true;

    //! Configuration for wrench distribution
    mc_rtc::Configuration wrenchDistConfig;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManager(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
   */
  virtual void reset();

  /** \brief Update.

      This method should be called once every control cycle.
   */
  virtual void update();

  /** \brief Const accessor to the configuration. */
  virtual const Configuration & config() const = 0;

  /** \brief Add entries to the GUI. */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Remove entries from the GUI. */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger);

  /** \brief Remove entries from the logger. */
  virtual void removeFromLogger(mc_rtc::Logger & logger);

  /** \brief Set anchor frame. */
  void setAnchorFrame();

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

  /** \brief Accessor to the configuration. */
  virtual Configuration & config() = 0;

  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() = 0;

  /** \brief Whether to assume that CoM Z is constant. */
  virtual bool isConstantComZ() const = 0;

  /** \brief Calculate anchor frame.
      \param robot robot
   */
  sva::PTransformd calcAnchorFrame(const mc_rbdyn::Robot & robot) const;

  /** \brief Calculate ZMP from wrench list.
      \param wrenchList wrench list
      \param zmpPlaneHeight height of ZMP plane
      \param zmpPlaneNormal normal of ZMP plane
  */
  Eigen::Vector3d calcZmp(const std::unordered_map<Foot, sva::ForceVecd> & wrenchList,
                          double zmpPlaneHeight = 0,
                          const Eigen::Vector3d & zmpPlaneNormal = Eigen::Vector3d::UnitZ()) const;

protected:
  //! Pointer to controller
  BaselineWalkingController * ctlPtr_ = nullptr;

  //! Robot mass [kg]
  double robotMass_ = 0;

  //! CoM used as the initial state of MPC
  Eigen::Vector3d mpcCom_ = Eigen::Vector3d::Zero();

  //! CoM velocity used as the initial state of MPC
  Eigen::Vector3d mpcComVel_ = Eigen::Vector3d::Zero();

  //! Reference ZMP
  Eigen::Vector3d refZmp_ = Eigen::Vector3d::Zero();

  //! ZMP planned by MPC
  Eigen::Vector3d plannedZmp_ = Eigen::Vector3d::Zero();

  //! Force Z planned by MPC
  double plannedForceZ_ = 0;

  //! ZMP with feedback control
  Eigen::Vector3d controlZmp_ = Eigen::Vector3d::Zero();

  //! Force Z with feedback control
  double controlForceZ_ = 0;

  //! Wrench distribution
  std::shared_ptr<WrenchDistribution> wrenchDist_;

  //! Contact list
  std::unordered_map<Foot, std::shared_ptr<Contact>> contactList_;
};
} // namespace BWC

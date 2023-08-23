#pragma once

#include <CCC/FootGuidedControl.h>

#include <BaselineWalkingController/CentroidalManager.h>

namespace BWC
{
/** \brief Centroidal manager with foot-guided control.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
*/
class CentroidalManagerFootGuidedControl : public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    //! Whether to reinitialize MPC when reference CoM Z position is updated
    bool reinitForRefComZ = true;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerFootGuidedControl(BaselineWalkingController * ctlPtr,
                                     const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Reset.

      This method should be called once when controller is reset.
   */
  virtual void reset() override;

  /** \brief Const accessor to the configuration. */
  inline virtual const Configuration & config() const override
  {
    return config_;
  }

  /** \brief Add entries to the logger. */
  virtual void addToLogger(mc_rtc::Logger & logger) override;

protected:
  /** \brief Accessor to the configuration. */
  inline virtual Configuration & config() override
  {
    return config_;
  }

  /** \brief Run MPC to plan centroidal trajectory.

      This method calculates plannedZmp_ and plannedForceZ_ from mpcCom_ and mpcComVel_.
   */
  virtual void runMpc() override;

  /** \brief Whether to assume that CoM Z is constant. */
  inline virtual bool isConstantComZ() const override
  {
    return true;
  }

  /** \brief Calculate reference data of MPC. */
  CCC::FootGuidedControl::RefData calcRefData() const;

protected:
  //! Configuration
  Configuration config_;

  //! Foot-guided control
  std::shared_ptr<CCC::FootGuidedControl> footGuided_;

  //! Reference CoM Z position of the previous control step
  double lastRefComZ_ = 0;
};
} // namespace BWC

#pragma once

#include <CCC/PreviewControlZmp.h>

#include <BaselineWalkingController/CentroidalManager.h>

namespace BWC
{
/** \brief Centroidal manager with preview control.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
*/
class CentroidalManagerPreviewControlZmp : virtual public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    //! Horizon duration [sec]
    double horizonDuration = 2.0;

    //! Horizon dt [sec]
    double horizonDt = 0.005;

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
  CentroidalManagerPreviewControlZmp(BaselineWalkingController * ctlPtr,
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
  virtual Eigen::Vector2d calcRefData(double t) const;

protected:
  //! Configuration
  Configuration config_;

  //! Preview control
  std::shared_ptr<CCC::PreviewControlZmp> pc_;

  //! Whether it is the first iteration
  bool firstIter_ = true;

  //! Reference CoM Z position of the previous control step
  double lastRefComZ_ = 0;
};
} // namespace BWC

#pragma once

#include <CCC/DdpZmp.h>

#include <BaselineWalkingController/CentroidalManager.h>

namespace BWC
{
/** \brief Centroidal manager with DDP.

    Centroidal manager calculates the centroidal targets from the specified reference ZMP trajectory and sensor
   measurements.
*/
class CentroidalManagerDdpZmp : public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    //! Horizon duration [sec]
    double horizonDuration = 2.0;

    //! Horizon dt [sec]
    double horizonDt = 0.02;

    //! DDP maximum iteration
    int ddpMaxIter = 1;

    //! Weight parameter of MPC objective function
    CCC::DdpZmp::WeightParam mpcWeightParam;

    /** \brief Load mc_rtc configuration. */
    virtual void load(const mc_rtc::Configuration & mcRtcConfig) override;
  };

public:
  /** \brief Constructor.
      \param ctlPtr pointer to controller
      \param mcRtcConfig mc_rtc configuration
   */
  CentroidalManagerDdpZmp(BaselineWalkingController * ctlPtr, const mc_rtc::Configuration & mcRtcConfig = {});

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
    return false;
  }

  /** \brief Calculate reference data of MPC. */
  CCC::DdpZmp::RefData calcRefData(double t) const;

protected:
  //! Configuration
  Configuration config_;

  //! DDP
  std::shared_ptr<CCC::DdpZmp> ddp_;
};
} // namespace BWC

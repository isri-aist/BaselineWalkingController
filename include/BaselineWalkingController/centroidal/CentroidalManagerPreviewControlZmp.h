#pragma once

#include <CCC/PreviewControlZmp.h>

#include <BaselineWalkingController/CentroidalManager.h>

namespace BWC
{
class CentroidalManagerPreviewControlZmp : public CentroidalManager
{
public:
  /** \brief Configuration. */
  struct Configuration : public CentroidalManager::Configuration
  {
    //! Horizon duration of DDP [sec]
    double horizonDuration = 2.0;

    //! Horizon dt of DDP [sec]
    double horizonDt = 0.02;

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
  Eigen::Vector2d calcRefData(double t) const;

protected:
  //! Configuration
  Configuration config_;

  //! Preview control for CoM trajectory generation
  std::shared_ptr<CCC::PreviewControlZmp> pc_;

  //! Whether it is the first iteration
  bool firstIter_ = true;
};
} // namespace BWC

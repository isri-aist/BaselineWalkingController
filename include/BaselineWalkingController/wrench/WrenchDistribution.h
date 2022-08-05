#pragma once

#include <qp_solver_collection/QpSolverCollection.h>

#include <BaselineWalkingController/FootTypes.h>

namespace BWC
{
class Contact;

/** \brief Wrench distribution. */
class WrenchDistribution
{
public:
  /** \brief Configuration. */
  struct Configuration
  {
    //! Wrench distribution weight
    sva::MotionVecd wrenchWeight = sva::MotionVecd(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0));

    //! Weight of QP regularization
    double regularWeight = 1e-8;

    //! Min/max ridge force
    std::pair<double, double> ridgeForceMinMax = std::make_pair(3, 1000); // [N]

    /** \brief Load mc_rtc configuration.
        \param mcRtcConfig mc_rtc configuration
    */
    void load(const mc_rtc::Configuration & mcRtcConfig);
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param contactList list of contact constraint
      \param mcRtcConfig mc_rtc configuration
   */
  WrenchDistribution(const std::unordered_map<Foot, std::shared_ptr<Contact>> & contactList,
                     const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Run wrench distribution calculation.
      \param desiredTotalWrench total wrench
      \param momentOrigin moment origin
      \returns total wrench of distributed result wrenches
   */
  sva::ForceVecd run(const sva::ForceVecd & desiredTotalWrench,
                     const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero());

  /** \brief Calculate wrench list.
      \param momentOrigin moment origin
      \returns contact wrench list
   */
  std::unordered_map<Foot, sva::ForceVecd> calcWrenchList(
      const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero()) const;

  /** \brief Const accessor to the configuration. */
  inline const Configuration & config() const noexcept
  {
    return config_;
  }

public:
  //! List of contact constraint
  std::unordered_map<Foot, std::shared_ptr<Contact>> contactList_;

  //! Result wrench ratio
  Eigen::VectorXd resultWrenchRatio_;

  //! Desired total wrench
  sva::ForceVecd desiredTotalWrench_ = sva::ForceVecd::Zero();

  //! Result total wrench
  sva::ForceVecd resultTotalWrench_ = sva::ForceVecd::Zero();

  //! QP solver
  std::shared_ptr<QpSolverCollection::QpSolver> qpSolver_;

  //! QP coefficients
  QpSolverCollection::QpCoeff qpCoeff_;

protected:
  //! Configuration
  Configuration config_;
};
} // namespace BWC

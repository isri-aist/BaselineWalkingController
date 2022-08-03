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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param contactList list of contact constraint
      \param mcRtcConfig mc_rtc configuration
   */
  WrenchDistribution(const std::unordered_map<Foot, std::shared_ptr<Contact>> & contactList,
                     const mc_rtc::Configuration & mcRtcConfig = {});

  /** \brief Run wrench distribution calculation.
      \param desiredTotalWrench total wrench
      \param origin origin position
      \returns total wrench of distributed result wrenches
   */
  sva::ForceVecd run(const sva::ForceVecd & desiredTotalWrench,
                     const Eigen::Vector3d & origin = Eigen::Vector3d::Zero());

  /** \brief Calculate wrench list.
      \param momentOrigin moment origin
      \returns unordered_map of contact name and contact wrench
   */
  std::unordered_map<Foot, sva::ForceVecd> calcWrenchList(
      const Eigen::Vector3d momentOrigin = Eigen::Vector3d::Zero()) const;

public:
  //! List of contact constraint
  std::unordered_map<Foot, std::shared_ptr<Contact>> contactList_;

  //! Result wrench ratio
  Eigen::VectorXd resultWrenchRatio_;

  //! Desired total wrench
  sva::ForceVecd desiredTotalWrench_ = sva::ForceVecd::Zero();

  //! Result total wrench
  sva::ForceVecd resultTotalWrench_ = sva::ForceVecd::Zero();

  //! Error between desired and result total wrenches
  sva::ForceVecd totalWrenchError_ = sva::ForceVecd::Zero();

  //! Wrench distribution weight
  sva::MotionVecd wrenchWeight_ = sva::MotionVecd(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0));

  //! Weight of QP regularization
  double regularWeight_ = 1e-8;

  //! Min/max ridge force
  std::pair<double, double> ridgeForceMinMax_ = std::make_pair(3, 1000); // [N]

  //! QP solver
  std::shared_ptr<QpSolverCollection::QpSolver> qpSolver_;

  //! QP coefficients
  QpSolverCollection::QpCoeff qpCoeff_;
};
} // namespace BWC

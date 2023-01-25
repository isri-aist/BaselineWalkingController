#pragma once

#include <mc_control/fsm/Controller.h>

#include <BaselineWalkingController/FootTypes.h>

namespace mc_tasks
{
class CoMTask;
class OrientationTask;
} // namespace mc_tasks

namespace BWC
{
class FootManager;
class CentroidalManager;
class FirstOrderImpedanceTask;

/** \brief Humanoid walking controller with various baseline methods. */
struct BaselineWalkingController : public mc_control::fsm::Controller
{
public:
  /** \brief Constructor. */
  BaselineWalkingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & _config);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Get controller name. */
  inline const std::string & name() const
  {
    return name_;
  }

  /** \brief Get current time. */
  inline double t() const noexcept
  {
    return t_;
  }

  /** \brief Get timestep. */
  inline double dt() const
  {
    return solver().dt();
  }

  /** \brief Set default anchor. */
  void setDefaultAnchor();

public:
  //! CoM task
  std::shared_ptr<mc_tasks::CoMTask> comTask_;

  //! Base link orientation task
  std::shared_ptr<mc_tasks::OrientationTask> baseOriTask_;

  //! Foot tasks
  std::unordered_map<Foot, std::shared_ptr<FirstOrderImpedanceTask>> footTasks_;

  //! Foot manager
  std::shared_ptr<FootManager> footManager_;

  //! Centroidal manager
  std::shared_ptr<CentroidalManager> centroidalManager_;

  //! Whether to enable manager update
  bool enableManagerUpdate_ = false;

protected:
  //! Controller name
  std::string name_ = "BWC";

  //! Current time [sec]
  double t_ = 0;
};
} // namespace BWC

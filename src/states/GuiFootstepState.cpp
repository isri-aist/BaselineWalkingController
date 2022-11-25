#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui/Form.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/GuiFootstepState.h>

using namespace BWC;

void GuiFootstepState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Load configuration
  if(config_.has("configs"))
  {
    if(config_("configs").has("deltaTransLimit"))
    {
      deltaTransLimit_ = config_("configs")("deltaTransLimit");
      deltaTransLimit_[2] = mc_rtc::constants::toRad(deltaTransLimit_[2]);
    }
  }

  // Setup GUI
  ctl().gui()->addElement(
      {ctl().name(), "GuiFootstep"},
      mc_rtc::gui::Form(
          "Walk",
          [this](const mc_rtc::Configuration & config) {
            if(ctl().footManager_->footstepQueue().size() > 0)
            {
              mc_rtc::log::error(
                  "[GuiFootstepState] The footstep command can be sent only when the footstep queue is empty: {}",
                  ctl().footManager_->footstepQueue().size());
              return;
            }
            sendFootstepList(Eigen::Vector3d(config(walkConfigKeys_.at("x")), config(walkConfigKeys_.at("y")),
                                             mc_rtc::constants::toRad(config(walkConfigKeys_.at("theta")))),
                             config(walkConfigKeys_.at("last")));
          },
          mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("x"), true, 0.0),
          mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("y"), true, 0.0),
          mc_rtc::gui::FormNumberInput(walkConfigKeys_.at("theta"), true, 0.0),
          mc_rtc::gui::FormIntegerInput(walkConfigKeys_.at("last"), true, 0)));
  ctl().gui()->addElement({ctl().name(), "GuiFootstep", "Config"},
                          mc_rtc::gui::ArrayInput(
                              "deltaTransLimit", {"x", "y", "theta"},
                              [this]() -> Eigen::Vector3d {
                                return Eigen::Vector3d(deltaTransLimit_[0], deltaTransLimit_[1],
                                                       mc_rtc::constants::toDeg(deltaTransLimit_[2]));
                              },
                              [this](const Eigen::Vector3d & v) {
                                deltaTransLimit_ = Eigen::Vector3d(v[0], v[1], mc_rtc::constants::toRad(v[2]));
                              }));

  output("OK");
}

bool GuiFootstepState::run(mc_control::fsm::Controller &)
{
  return false;
}

void GuiFootstepState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "GuiFootstep"});
}

void GuiFootstepState::sendFootstepList(const Eigen::Vector3d & goalTrans, // (x [m], y [m], theta [rad])
                                        int lastFootstepNum)
{
  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  // The 2D variables (i.e., goalTrans, deltaTrans) represent the transformation relative to the initial pose,
  // while the 3D variables (i.e., initialFootMidpose, goalFootMidpose, footMidpose) represent the
  // transformation in the world frame.
  const sva::PTransformd & initialFootMidpose = projGround(sva::interpolate(
      ctl().footManager_->targetFootPose(Foot::Left), ctl().footManager_->targetFootPose(Foot::Right), 0.5));
  const sva::PTransformd & goalFootMidpose = convertTo3d(goalTrans) * initialFootMidpose;

  Foot foot = goalTrans.y() >= 0 ? Foot::Left : Foot::Right;
  sva::PTransformd footMidpose = initialFootMidpose;
  double startTime = ctl().t() + 1.0;

  while(convertTo2d(goalFootMidpose * footMidpose.inv()).norm() > 1e-6)
  {
    Eigen::Vector3d deltaTransMax = deltaTransLimit_;
    Eigen::Vector3d deltaTransMin = -deltaTransLimit_;
    if(foot == Foot::Left)
    {
      deltaTransMin.y() = 0;
    }
    else
    {
      deltaTransMax.y() = 0;
    }
    Eigen::Vector3d deltaTrans = convertTo2d(goalFootMidpose * footMidpose.inv());
    mc_filter::utils::clampInPlace(deltaTrans, deltaTransMin, deltaTransMax);
    footMidpose = convertTo3d(deltaTrans) * footMidpose;

    const auto & footstep = ctl().footManager_->makeFootstep(foot, footMidpose, startTime);
    ctl().footManager_->appendFootstep(footstep);

    foot = opposite(foot);
    startTime = footstep.transitEndTime;
  }

  for(int i = 0; i < lastFootstepNum + 1; i++)
  {
    const auto & footstep = ctl().footManager_->makeFootstep(foot, footMidpose, startTime);
    ctl().footManager_->appendFootstep(footstep);

    foot = opposite(foot);
    startTime = footstep.transitEndTime;
  }
}

EXPORT_SINGLE_STATE("BWC::GuiFootstep", GuiFootstepState)

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/ros.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/TeleopState.h>

using namespace BWC;

void TeleopState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Skip if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    output("OK");
    return;
  }

  // Load configuration
  std::string twistTopicName = "/cmd_vel";
  if(config_.has("configs"))
  {
    if(config_("configs").has("deltaTransLimit"))
    {
      deltaTransLimit_ = config_("configs")("deltaTransLimit");
      deltaTransLimit_[2] = mc_rtc::constants::toRad(deltaTransLimit_[2]);
    }
    if(config_("configs").has("velScale"))
    {
      velScale_ = config_("configs")("velScale");
      velScale_[2] = mc_rtc::constants::toRad(velScale_[2]);
    }
    config_("configs")("footstepQueueSize", footstepQueueSize_);
    config_("configs")("twistTopicName", twistTopicName);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  twistSub_ = nh_->subscribe<geometry_msgs::Twist>(twistTopicName, 1, &TeleopState::twistCallback, this);

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "Teleop"},
                          mc_rtc::gui::Button("StartTeleop", [this]() { startTriggered_ = true; }));
  ctl().gui()->addElement({ctl().name(), "Teleop", "State"},
                          mc_rtc::gui::ArrayInput(
                              "targetDeltaTrans", {"x", "y", "theta"},
                              [this]() -> Eigen::Vector3d {
                                return Eigen::Vector3d(targetDeltaTrans_[0], targetDeltaTrans_[1],
                                                       mc_rtc::constants::toDeg(targetDeltaTrans_[2]));
                              },
                              [this](const Eigen::Vector3d & v) {
                                targetDeltaTrans_ = Eigen::Vector3d(v[0], v[1], mc_rtc::constants::toRad(v[2]));
                              }));
  ctl().gui()->addElement({ctl().name(), "Teleop", "Config"},
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

bool TeleopState::run(mc_control::fsm::Controller &)
{
  // Finish if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    return true;
  }

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  // Process start/end trigger
  if(startTriggered_)
  {
    startTriggered_ = false;

    if(ctl().footManager_->footstepQueue().size() > 0)
    {
      mc_rtc::log::error("[TeleopState] Teleoperation can be started only when the footstep queue is empty: {}",
                         ctl().footManager_->footstepQueue().size());
    }
    else
    {
      startTeleop();
      ctl().gui()->removeElement({ctl().name(), "Teleop"}, "StartTeleop");
      ctl().gui()->addElement({ctl().name(), "Teleop"},
                              mc_rtc::gui::Button("EndTeleop", [this]() { endTriggered_ = true; }));
    }
  }
  if(endTriggered_)
  {
    endTriggered_ = false;
    endTeleop();
    ctl().gui()->removeElement({ctl().name(), "Teleop"}, "EndTeleop");
    ctl().gui()->addElement({ctl().name(), "Teleop"},
                            mc_rtc::gui::Button("StartTeleop", [this]() { startTriggered_ = true; }));
  }

  if(!teleopRunning_)
  {
    return false;
  }

  // Update footstep queue
  {
    auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
      return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
    };

    auto & footstepQueue = ctl().footManager_->footstepQueue();
    // Do not change the next footstep
    // Delete the second and subsequent footsteps, and add new ones
    footstepQueue.erase(footstepQueue.begin() + 1, footstepQueue.end());
    const auto & nextFootstep = footstepQueue.front();

    // Append new footstep
    Foot foot = opposite(nextFootstep.foot);
    sva::PTransformd footMidpose = projGround(
        sva::interpolate(ctl().footManager_->targetFootPose(opposite(nextFootstep.foot)), nextFootstep.pose, 0.5));
    double startTime = nextFootstep.transitEndTime;
    for(int i = 0; i < footstepQueueSize_ - 1; i++)
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
      Eigen::Vector3d deltaTrans = mc_filter::utils::clamp(targetDeltaTrans_, deltaTransMin, deltaTransMax);
      footMidpose = convertTo3d(deltaTrans) * footMidpose;

      const auto & footstep = ctl().footManager_->makeFootstep(foot, footMidpose, startTime);
      footstepQueue.push_back(footstep);

      foot = opposite(foot);
      startTime = footstep.transitEndTime;
    }
  }

  return false;
}

void TeleopState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "Teleop"});
}

void TeleopState::startTeleop()
{
  teleopRunning_ = true;

  targetDeltaTrans_.setZero();

  // Add footsteps to queue for walking in place
  Foot foot = Foot::Left;
  const sva::PTransformd & footMidpose = projGround(sva::interpolate(
      ctl().footManager_->targetFootPose(Foot::Left), ctl().footManager_->targetFootPose(Foot::Right), 0.5));
  double startTime = ctl().t() + 1.0;
  for(int i = 0; i < footstepQueueSize_; i++)
  {
    const auto & footstep = ctl().footManager_->makeFootstep(foot, footMidpose, startTime);
    ctl().footManager_->appendFootstep(footstep);

    foot = opposite(foot);
    startTime = footstep.transitEndTime;
  }
}

void TeleopState::endTeleop()
{
  teleopRunning_ = false;

  targetDeltaTrans_.setZero();

  // Update last footstep pose to align both feet
  const auto & footManagerConfig = ctl().footManager_->config();
  const auto & lastFootstep1 = *(ctl().footManager_->footstepQueue().rbegin() + 1);
  auto & lastFootstep2 = *(ctl().footManager_->footstepQueue().rbegin());
  sva::PTransformd footMidpose = footManagerConfig.midToFootTranss.at(lastFootstep1.foot).inv() * lastFootstep1.pose;
  lastFootstep2.pose = footManagerConfig.midToFootTranss.at(lastFootstep2.foot) * footMidpose;
}

void TeleopState::twistCallback(const geometry_msgs::Twist::ConstPtr & twistMsg)
{
  targetDeltaTrans_ =
      velScale_.cwiseProduct(Eigen::Vector3d(twistMsg->linear.x, twistMsg->linear.y, twistMsg->angular.z));
}

EXPORT_SINGLE_STATE("BWC::Teleop", TeleopState)

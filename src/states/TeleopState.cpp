#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/TeleopState.h>

using namespace BWC;

void TeleopState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Load configuration
  std::string twistTopicName = "/cmd_vel";
  if(config_.has("configs"))
  {
    config_("configs")("twistTopicName", twistTopicName);
    config_("configs")("footstepQueueSize", footstepQueueSize_);
    config_("configs")("velScale", velScale_);
    if(config_("configs").has("deltaTransLimit"))
    {
      deltaTransLimit_ = config_("configs")("deltaTransLimit");
      deltaTransLimit_[2] = mc_rtc::constants::toRad(deltaTransLimit_[2]);
    }
  }

  // Setup ROS
  nh_ = std::make_shared<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  twistSub_ = nh_->subscribe<geometry_msgs::Twist>(twistTopicName, 1, &TeleopState::twistCallback, this);

  // Setup GUI
  ctl().gui()->addElement({"BWC", "Teleop"}, mc_rtc::gui::Button("StartTeleop", [this]() { startTriggered_ = true; }));
  ctl().gui()->addElement(
      {"BWC", "Teleop", "Velocity"},
      mc_rtc::gui::ArrayInput(
          "TargetVel", {"x", "y", "yaw"}, [this]() { return targetVel_; }, [this](const Eigen::Vector3d & v) {}));

  output("OK");
}

bool TeleopState::run(mc_control::fsm::Controller &)
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  // Process start/end trigger
  if(startTriggered_)
  {
    startTriggered_ = false;
    startTeleop();
    ctl().gui()->removeElement({"BWC", "Teleop"}, "StartTeleop");
    ctl().gui()->addElement({"BWC", "Teleop"}, mc_rtc::gui::Button("EndTeleop", [this]() { endTriggered_ = true; }));
  }
  if(endTriggered_)
  {
    endTriggered_ = false;
    endTeleop();
    ctl().gui()->removeElement({"BWC", "Teleop"}, "EndTeleop");
    ctl().gui()->addElement({"BWC", "Teleop"},
                            mc_rtc::gui::Button("StartTeleop", [this]() { startTriggered_ = true; }));
  }

  if(!runTeleop_)
  {
    return false;
  }

  // Update footstep queue
  {
    auto & footstepQueue = ctl().footManager_->footstepQueue();
    // Do not change the next footstep
    // Delete the second and subsequent footsteps, and add new ones
    footstepQueue.erase(footstepQueue.begin() + 1, footstepQueue.end());
    const auto & nextFootstep = footstepQueue.front();

    Foot foot = opposite(nextFootstep.foot);
    sva::PTransformd footMidpose = projGround(
        sva::interpolate(ctl().footManager_->targetFootPose(opposite(nextFootstep.foot)), nextFootstep.pose, 0.5));
    double startTime = nextFootstep.transitEndTime;
    for(int i = 0; i < footstepQueueSize_ - 1; i++)
    {
      // Truncate inside movement
      Eigen::Vector3d targetVelWithLimit = targetVel_;
      if(foot == Foot::Left)
      {
        targetVelWithLimit.y() = std::max(targetVelWithLimit.y(), 0.0);
      }
      else
      {
        targetVelWithLimit.y() = std::min(targetVelWithLimit.y(), 0.0);
      }
      // Calculate next footMidpose from previous one and append new footstep
      sva::PTransformd deltaFootMidpose(sva::RotZ(targetVelWithLimit[2]),
                                        Eigen::Vector3d(targetVelWithLimit[0], targetVelWithLimit[1], 0));
      footMidpose = deltaFootMidpose * footMidpose;
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
  ctl().gui()->removeCategory({"BWC", "Teleop"});
}

void TeleopState::startTeleop()
{
  runTeleop_ = true;

  targetVel_.setZero();

  // Add footsteps to queue for walking in place
  Foot foot = Foot::Left;
  sva::PTransformd footMidpose = projGround(sva::interpolate(ctl().footManager_->targetFootPose(Foot::Left),
                                                             ctl().footManager_->targetFootPose(Foot::Right), 0.5));
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
  runTeleop_ = false;

  targetVel_.setZero();

  // Update footstep pose to align both feet
  auto & footstepQueue = ctl().footManager_->footstepQueue();
  const auto & nextFootstep = footstepQueue.front();
  auto & finalFootstep1 = *(footstepQueue.rbegin() + 1);
  auto & finalFootstep2 = *(footstepQueue.rbegin());
  sva::PTransformd footMidpose = projGround(
      sva::interpolate(ctl().footManager_->targetFootPose(opposite(nextFootstep.foot)), nextFootstep.pose, 0.5));
  const auto & footManagerConfig = ctl().footManager_->config();
  finalFootstep1.pose = footManagerConfig.midToFootTranss.at(finalFootstep1.foot) * footMidpose;
  finalFootstep2.pose = footManagerConfig.midToFootTranss.at(finalFootstep2.foot) * footMidpose;
}

void TeleopState::twistCallback(const geometry_msgs::Twist::ConstPtr & twistMsg)
{
  targetVel_ = velScale_.cwiseProduct(Eigen::Vector3d(twistMsg->linear.x, twistMsg->linear.y, twistMsg->angular.z));
}

EXPORT_SINGLE_STATE("BWC::Teleop", TeleopState)

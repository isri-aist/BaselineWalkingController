#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/XYTheta.h>
#include <mc_rtc/ros.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/RosWalkState.h>

using namespace BWC;

void RosWalkState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Skip if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    mc_rtc::log::error("[RosWalkState] ROS is not initialized.");
    output("OK");
    return;
  }

  // Load configuration
  std::string poseTopicName = "/goal_pose";
  if(config_.has("configs"))
  {
    if(config_("configs").has("goalOffset"))
    {
      goalOffset_ = config_("configs")("goalOffset");
      goalOffset_[2] = mc_rtc::constants::toRad(goalOffset_[2]);
    }
    config_("configs")("poseTopicName", poseTopicName);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  poseSub_ = nh_->subscribe<geometry_msgs::PoseStamped>(poseTopicName, 1, &RosWalkState::poseCallback, this);
  poseMsg_ = nullptr;

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "RosWalk"}, mc_rtc::gui::Button("SetGoal", [this]() { setGoal(); }),
                          mc_rtc::gui::Button("WalkToGoal", [this]() { walkToGoal(); }));

  output("OK");
}

bool RosWalkState::run(mc_control::fsm::Controller &)
{
  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  return false;
}

void RosWalkState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "RosWalk"});
}

void RosWalkState::setGoal()
{
  if(!poseMsg_)
  {
    mc_rtc::log::warning("[RosWalkState] ROS pose message has not been received.");
    return;
  }

  const auto & posMsg = poseMsg_->pose.position;
  const auto & oriMsg = poseMsg_->pose.orientation;
  sva::PTransformd goalFootMidpose = sva::PTransformd(
      Eigen::Quaterniond(oriMsg.w, oriMsg.x, oriMsg.y, oriMsg.z).normalized().toRotationMatrix().transpose(),
      Eigen::Vector3d(posMsg.x, posMsg.y, posMsg.z));
  const std::string & frameName = poseMsg_->header.frame_id;
  if(std::find(worldFrameNames_.begin(), worldFrameNames_.end(), frameName) != worldFrameNames_.end())
  {
    // Do nothing
  }
  else if(ctl().robot().hasFrame(frameName))
  {
    goalFootMidpose = goalFootMidpose * ctl().robot().frame(frameName).position();
  }
  else
  {
    mc_rtc::log::error("[RosWalkState] frame_id in pose message is unknown: {}.", frameName);
    return;
  }

  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };
  goalFootMidTrans_ = convertTo2d(convertTo3d(goalOffset_) * goalFootMidpose);

  ctl().gui()->removeElement({ctl().name(), "RosWalk"}, "GoalPose");
  ctl().gui()->addElement(
      {ctl().name(), "RosWalk"},
      mc_rtc::gui::XYTheta(
          "GoalPose",
          [this]() -> std::array<double, 4> {
            std::array<double, 4> goalFootMidTrans;
            std::copy(goalFootMidTrans_.data(), goalFootMidTrans_.data() + 3, goalFootMidTrans.begin());
            goalFootMidTrans[3] = 0.0;
            return goalFootMidTrans;
          },
          [this](const std::array<double, 4> & goalFootMidTrans) {
            std::copy(goalFootMidTrans.begin(), goalFootMidTrans.begin() + 3, goalFootMidTrans_.data());
          }));

  poseMsg_ = nullptr;
}

void RosWalkState::walkToGoal()
{
  if(!ctl().gui()->hasElement({ctl().name(), "RosWalk"}, "GoalPose"))
  {
    mc_rtc::log::warning("[RosWalkState] Set the goal pose in advance.");
    return;
  }

  auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
    return Eigen::Vector3d(pose.translation().x(), pose.translation().y(), mc_rbdyn::rpyFromMat(pose.rotation()).z());
  };
  auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
    return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
  };

  sva::PTransformd currentFootMidpose = projGround(sva::interpolate(
      ctl().footManager_->targetFootPose(Foot::Left), ctl().footManager_->targetFootPose(Foot::Right), 0.5));
  Eigen::Vector3d targetTrans = convertTo2d(convertTo3d(goalFootMidTrans_) * currentFootMidpose.inv());
  ctl().footManager_->walkToRelativePose(targetTrans);

  ctl().gui()->removeElement({ctl().name(), "RosWalk"}, "GoalPose");
}

void RosWalkState::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & poseMsg)
{
  poseMsg_ = std::make_shared<geometry_msgs::PoseStamped>(*poseMsg);
}

EXPORT_SINGLE_STATE("BWC::RosWalk", RosWalkState)

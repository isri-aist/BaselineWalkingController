#include <mc_rtc/gui/Button.h>
#include <mc_rtc/ros.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/TeleopState.h>

using namespace BWC;

void TeleopState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Skip if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    mc_rtc::log::error("[TeleopState] ROS is not initialized.");
    output("OK");
    return;
  }

  // Load configuration
  std::string twistTopicName = "/cmd_vel";
  if(config_.has("configs"))
  {
    if(config_("configs").has("velScale"))
    {
      velScale_ = config_("configs")("velScale");
      velScale_[2] = mc_rtc::constants::toRad(velScale_[2]);
    }
    config_("configs")("twistTopicName", twistTopicName);
  }

  // Setup ROS
  rclcpp::NodeOptions node_options;
  nh_ = rclcpp::Node::make_shared("teleop_state", node_options);
  // Use a dedicated queue so as not to call callbacks of other modules
  twistSub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(twistTopicName, 1, std::bind(&TeleopState::twistCallback, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);

  // Setup GUI
  ctl().gui()->addElement({ctl().name(), "Teleop"},
                          mc_rtc::gui::Button("StartTeleop", [this]() { ctl().footManager_->startVelMode(); }));
  ctl().gui()->addElement({ctl().name(), "Teleop", "State"},
                          mc_rtc::gui::ArrayInput(
                              "targetVel", {"x", "y", "theta"},
                              [this]() -> Eigen::Vector3d {
                                return Eigen::Vector3d(targetVel_[0], targetVel_[1],
                                                       mc_rtc::constants::toDeg(targetVel_[2]));
                              },
                              [this](const Eigen::Vector3d & v) {
                                targetVel_ = Eigen::Vector3d(v[0], v[1], mc_rtc::constants::toRad(v[2]));
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
  executor_->spin_once(std::chrono::seconds(0));

  // Update GUI
  bool velMode = ctl().footManager_->velModeEnabled();
  if(velMode && ctl().gui()->hasElement({ctl().name(), "Teleop"}, "StartTeleop"))
  {
    ctl().gui()->addElement({ctl().name(), "Teleop"},
                            mc_rtc::gui::Button("EndTeleop", [this]() { ctl().footManager_->endVelMode(); }));
    ctl().gui()->removeElement({ctl().name(), "Teleop"}, "StartTeleop");
  }
  else if(!velMode && ctl().gui()->hasElement({ctl().name(), "Teleop"}, "EndTeleop"))
  {
    ctl().gui()->addElement({ctl().name(), "Teleop"},
                            mc_rtc::gui::Button("StartTeleop", [this]() { ctl().footManager_->startVelMode(); }));
    ctl().gui()->removeElement({ctl().name(), "Teleop"}, "EndTeleop");
  }

  // Set target velocity
  if(ctl().footManager_->velModeEnabled())
  {
    ctl().footManager_->setRelativeVel(targetVel_);
  }

  return false;
}

void TeleopState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "Teleop"});
}

void TeleopState::twistCallback(const geometry_msgs::msg::Twist::ConstSharedPtr & twistMsg)
{
  targetVel_ = velScale_.cwiseProduct(Eigen::Vector3d(twistMsg->linear.x, twistMsg->linear.y, twistMsg->angular.z));
}

EXPORT_SINGLE_STATE("BWC::Teleop", TeleopState)

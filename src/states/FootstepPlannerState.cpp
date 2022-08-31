#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/XYTheta.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/MathUtils.h>
#include <BaselineWalkingController/states/FootstepPlannerState.h>

using namespace BWC;

BFP::FootstepEnvConfigMcRtc::FootstepEnvConfigMcRtc(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("theta_divide_num", theta_divide_num);
  mcRtcConfig("xy_divide_step", xy_divide_step);

  mcRtcConfig("cost_scale", cost_scale);
  mcRtcConfig("cost_theta_scale", cost_theta_scale);
  mcRtcConfig("step_cost", step_cost);

  if(mcRtcConfig.has("heuristic_type"))
  {
    heuristic_type = BFP::strToHeuristicType(mcRtcConfig("heuristic_type"));
  }
  mcRtcConfig("dijkstra_path_heuristic_expand_scale", dijkstra_path_heuristic_expand_scale);
  mc_rtc::log::info("[FootstepEnvConfigMcRtc] Heuristic type: {}", std::to_string(heuristic_type));

  mcRtcConfig("nominal_foot_separation", nominal_foot_separation);

  if(mcRtcConfig.has("r2l_action_cont_list"))
  {
    r2l_action_cont_list.clear();
    for(const auto & r2l_action_cont_config : mcRtcConfig("r2l_action_cont_list"))
    {
      if(r2l_action_cont_config.size() != 3)
      {
        throw std::invalid_argument("Invalid r2l_action_cont_list. Size of each element should be 3.");
      }
      r2l_action_cont_list.emplace_back(r2l_action_cont_config[0], r2l_action_cont_config[1],
                                        r2l_action_cont_config[2]);
    }
  }
  mc_rtc::log::info("[FootstepEnvConfigMcRtc] Number of actions: {}", r2l_action_cont_list.size());

  if(mcRtcConfig.has("r2l_reachable_min"))
  {
    const auto & r2l_reachable_min_config = mcRtcConfig("r2l_reachable_min");
    if(r2l_reachable_min_config.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_min. Size of element should be 3.");
    }
    r2l_reachable_min =
        FootstepActionCont(r2l_reachable_min_config[0], r2l_reachable_min_config[1], r2l_reachable_min_config[2]);
  }
  if(mcRtcConfig.has("r2l_reachable_max"))
  {
    const auto & r2l_reachable_max_config = mcRtcConfig("r2l_reachable_max");
    if(r2l_reachable_max_config.size() != 3)
    {
      throw std::invalid_argument("Invalid r2l_reachable_max. Size of element should be 3.");
    }
    r2l_reachable_max =
        FootstepActionCont(r2l_reachable_max_config[0], r2l_reachable_max_config[1], r2l_reachable_max_config[2]);
  }

  if(mcRtcConfig.has("rect_obstacle_list"))
  {
    rect_obst_list.clear();
    for(const auto & rect_obst_config : mcRtcConfig("rect_obstacle_list"))
    {
      if(rect_obst_config.size() != 4)
      {
        throw std::invalid_argument("Invalid rect_obstacle_list. Size of each element should be 4.");
      }
      rect_obst_list.emplace_back(rect_obst_config[0], rect_obst_config[1], rect_obst_config[2], rect_obst_config[3]);
    }
  }
  mc_rtc::log::info("[FootstepEnvConfigMcRtc] Number of obstacles: {}", rect_obst_list.size());
}

void FootstepPlannerState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Setup footstep planner
  planner_ = std::make_shared<BFP::FootstepPlanner>(
      std::make_shared<BFP::FootstepEnvConfigMcRtc>(config_("footstepPlanner", mc_rtc::Configuration())));

  // Setup GUI
  ctl().gui()->addElement(
      {"BWC", "FootstepPlanner"}, mc_rtc::gui::Button("PlanAndWalk", [this]() { triggered_ = true; }),
      mc_rtc::gui::XYTheta(
          "GoalPose", [this]() -> std::array<double, 3> { return goalFootMidpose_; },
          [this](const std::array<double, 4> & goalFootMidpose) {
            return std::copy(goalFootMidpose.begin(), goalFootMidpose.begin() + 3, goalFootMidpose_.begin());
          }));

  output("OK");
}

bool FootstepPlannerState::run(mc_control::fsm::Controller &)
{
  if(triggered_)
  {
    triggered_ = false;

    if(ctl().footManager_->footstepQueue().size() > 0)
    {
      mc_rtc::log::error(
          "[FootstepPlannerState] Planning and walking can be started only when the footstep queue is empty: {}",
          ctl().footManager_->footstepQueue().size());
    }
    else
    {
      // TODO
      mc_rtc::log::success("goalFootMidpose: [{}, {}, {}]", goalFootMidpose_[0], goalFootMidpose_[1],
                           goalFootMidpose_[2]);
    }
  }

  return false;
}

void FootstepPlannerState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({"BWC", "FootstepPlanner"});
}

EXPORT_SINGLE_STATE("BWC::FootstepPlanner", FootstepPlannerState)

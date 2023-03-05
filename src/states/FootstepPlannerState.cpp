#include <chrono>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Polygon.h>
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
  mc_rtc::Configuration footstepPlannerConfig;
  if(config_.has("configs"))
  {
    config_("configs")("autoStart", triggered_);
    config_("configs")("goalFootMidpose", goalFootMidpose_);
    config_("configs")("maxPlanningDuration", maxPlanningDuration_);
    config_("configs")("initialHeuristicsWeight", initialHeuristicsWeight_);
    config_("configs")("footstepPlanner", footstepPlannerConfig);
  }
  footstepPlanner_ =
      std::make_shared<BFP::FootstepPlanner>(std::make_shared<BFP::FootstepEnvConfigMcRtc>(footstepPlannerConfig));

  // Setup GUI
  std::vector<std::vector<Eigen::Vector3d>> obstPolygonList;
  for(const auto & rect_obst : footstepPlanner_->env_->config()->rect_obst_list)
  {
    double x_center = rect_obst.x_center;
    double y_center = rect_obst.y_center;
    double x_half_length = rect_obst.x_half_length;
    double y_half_length = rect_obst.y_half_length;
    obstPolygonList.push_back({Eigen::Vector3d(x_center + x_half_length, y_center + y_half_length, 0),
                               Eigen::Vector3d(x_center - x_half_length, y_center + y_half_length, 0),
                               Eigen::Vector3d(x_center - x_half_length, y_center - y_half_length, 0),
                               Eigen::Vector3d(x_center + x_half_length, y_center - y_half_length, 0)});
  }
  ctl().gui()->addElement(
      {ctl().name(), "FootstepPlanner"}, mc_rtc::gui::Button("PlanAndWalk", [this]() { triggered_ = true; }),
      mc_rtc::gui::XYTheta(
          "GoalPose",
          [this]() -> std::array<double, 4> {
            std::array<double, 4> goalFootMidpose;
            std::copy(goalFootMidpose_.begin(), goalFootMidpose_.begin() + 3, goalFootMidpose.begin());
            goalFootMidpose[3] = 0.0;
            return goalFootMidpose;
          },
          [this](const std::array<double, 4> & goalFootMidpose) {
            std::copy(goalFootMidpose.begin(), goalFootMidpose.begin() + 3, goalFootMidpose_.begin());
          }),
      mc_rtc::gui::Polygon("Obstacles", {mc_rtc::gui::Color::Gray, 0.02},
                           [obstPolygonList]() { return obstPolygonList; }));

  // Setup thread
  planningThread_ = std::thread(&FootstepPlannerState::planningThread, this);

  output("OK");
}

bool FootstepPlannerState::run(mc_control::fsm::Controller &)
{
  return false;
}

void FootstepPlannerState::teardown(mc_control::fsm::Controller &)
{
  // Clean up GUI
  ctl().gui()->removeCategory({ctl().name(), "FootstepPlanner"});

  // Clean up thread
  running_ = false;
  if(planningThread_.joinable())
  {
    planningThread_.join();
  }
}

void FootstepPlannerState::planningThread()
{
  while(running_)
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
        auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d {
          return Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                                 mc_rbdyn::rpyFromMat(pose.rotation()).z());
        };
        auto convertTo3d = [](const Eigen::Vector3d & trans) -> sva::PTransformd {
          return sva::PTransformd(sva::RotZ(trans.z()), Eigen::Vector3d(trans.x(), trans.y(), 0));
        };

        std::unordered_map<Foot, Eigen::Vector3d> footPoses2d = {
            {Foot::Left, convertTo2d(ctl().footManager_->targetFootPose(Foot::Left))},
            {Foot::Right, convertTo2d(ctl().footManager_->targetFootPose(Foot::Right))}};
        footstepPlanner_->setStartGoal(
            std::make_shared<BFP::FootstepState>(footstepPlanner_->env_->contToDiscXy(footPoses2d.at(Foot::Left)[0]),
                                                 footstepPlanner_->env_->contToDiscXy(footPoses2d.at(Foot::Left)[1]),
                                                 footstepPlanner_->env_->contToDiscTheta(footPoses2d.at(Foot::Left)[2]),
                                                 BFP::Foot::LEFT),
            std::make_shared<BFP::FootstepState>(
                footstepPlanner_->env_->contToDiscXy(footPoses2d.at(Foot::Right)[0]),
                footstepPlanner_->env_->contToDiscXy(footPoses2d.at(Foot::Right)[1]),
                footstepPlanner_->env_->contToDiscTheta(footPoses2d.at(Foot::Right)[2]), BFP::Foot::RIGHT),
            footstepPlanner_->env_->makeStateFromMidpose(goalFootMidpose_, BFP::Foot::LEFT),
            footstepPlanner_->env_->makeStateFromMidpose(goalFootMidpose_, BFP::Foot::RIGHT));
        footstepPlanner_->run(false, maxPlanningDuration_, initialHeuristicsWeight_);

        if(footstepPlanner_->solution_.is_solved)
        {
          double startTime = ctl().t() + 1.0;
          for(auto it = footstepPlanner_->solution_.state_list.begin() + 2;
              it != footstepPlanner_->solution_.state_list.end(); it++)
          {
            Foot foot = ((*it)->foot_ == BFP::Foot::LEFT ? Foot::Left : Foot::Right);
            sva::PTransformd pose = convertTo3d(Eigen::Vector3d(
                footstepPlanner_->env_->discToContXy((*it)->x_), footstepPlanner_->env_->discToContXy((*it)->y_),
                footstepPlanner_->env_->discToContTheta((*it)->theta_)));
            Footstep footstep(foot, pose, startTime,
                              startTime
                                  + 0.5 * ctl().footManager_->config().doubleSupportRatio
                                        * ctl().footManager_->config().footstepDuration,
                              startTime
                                  + (1.0 - 0.5 * ctl().footManager_->config().doubleSupportRatio)
                                        * ctl().footManager_->config().footstepDuration,
                              startTime + ctl().footManager_->config().footstepDuration);
            ctl().footManager_->appendFootstep(footstep);
            startTime = footstep.transitEndTime;
          }
        }
        else
        {
          mc_rtc::log::error("[FootstepPlannerState] Failed footstep planning.");
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

EXPORT_SINGLE_STATE("BWC::FootstepPlanner", FootstepPlannerState)

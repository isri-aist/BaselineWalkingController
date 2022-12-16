add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(ConfigFootstepState ConfigFootstepState.cpp)
target_link_libraries(ConfigFootstepState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(GuiFootstepState GuiFootstepState.cpp)
target_link_libraries(GuiFootstepState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(TeleopState TeleopState.cpp)
target_link_libraries(TeleopState PUBLIC
  ${CONTROLLER_NAME})

find_package(baseline_footstep_planner QUIET)
if(${baseline_footstep_planner_FOUND})
  message("-- Build FootstepPlannerState as baseline_footstep_planner found")
  add_project_dependency(baseline_footstep_planner REQUIRED)
  add_fsm_state(FootstepPlannerState FootstepPlannerState.cpp)
  target_link_libraries(FootstepPlannerState PUBLIC baseline_footstep_planner::BaselineFootstepPlanner ${CONTROLLER_NAME})
else()
  message("-- Not build FootstepPlannerState as baseline_footstep_planner not found")
endif()

add_fsm_data_directory(data)

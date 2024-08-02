add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(ConfigWalkState ConfigWalkState.cpp)
target_link_libraries(ConfigWalkState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(GuiWalkState GuiWalkState.cpp)
target_link_libraries(GuiWalkState PUBLIC
  ${CONTROLLER_NAME})

if(DEFINED CATKIN_DEVEL_PREFIX)
  add_fsm_state(RosWalkState RosWalkState.cpp)
  target_link_libraries(RosWalkState PUBLIC
    ${CONTROLLER_NAME})

  add_fsm_state(TeleopState TeleopState.cpp)
  target_link_libraries(TeleopState PUBLIC
    ${CONTROLLER_NAME})
endif()

find_package(baseline_footstep_planner QUIET)
if(${baseline_footstep_planner_FOUND})
  message("-- Build FootstepPlannerState as baseline_footstep_planner found")
  add_project_dependency(baseline_footstep_planner REQUIRED)
  add_fsm_state(FootstepPlannerState FootstepPlannerState.cpp)
  if(DEFINED CATKIN_DEVEL_PREFIX)
    target_include_directories(FootstepPlannerState PUBLIC
      ${baseline_footstep_planner_INCLUDE_DIRS})
    target_link_directories(FootstepPlannerState PUBLIC
      ${baseline_footstep_planner_LIBRARY_DIRS})
    target_link_libraries(FootstepPlannerState PUBLIC
      ${baseline_footstep_planner_LIBRARIES}
      ${CONTROLLER_NAME})
  else()
    target_link_libraries(FootstepPlannerState PUBLIC baseline_footstep_planner::BaselineFootstepPlanner ${CONTROLLER_NAME})
  endif()
  file(WRITE "${PROJECT_BINARY_DIR}/generated/FootstepPlannerConfig.cmake" "set(baseline_walking_controller_FootstepPlanner_FOUND TRUE)")
  install(FILES "${PROJECT_BINARY_DIR}/generated/FootstepPlannerConfig.cmake" DESTINATION "${CONFIG_INSTALL_DIR}")
else()
  message("-- Not build FootstepPlannerState as baseline_footstep_planner not found")
endif()

add_fsm_data_directory(data)

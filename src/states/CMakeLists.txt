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

add_fsm_state(FootstepPlannerState FootstepPlannerState.cpp)
target_link_libraries(FootstepPlannerState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_data_directory(data)

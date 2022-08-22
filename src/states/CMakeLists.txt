add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(ConfigFootstepState ConfigFootstepState.cpp)
target_link_libraries(ConfigFootstepState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(CommandState CommandState.cpp)
target_link_libraries(CommandState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(TeleopState TeleopState.cpp)
target_link_libraries(TeleopState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_data_directory(data)

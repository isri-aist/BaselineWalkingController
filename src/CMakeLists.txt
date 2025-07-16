set(CONTROLLER_NAME BaselineWalkingController)

add_library(${CONTROLLER_NAME} SHARED
  BaselineWalkingController.cpp
  MathUtils.cpp
  RobotUtils.cpp
  FootTypes.cpp
  FootManager.cpp
  CentroidalManager.cpp
  centroidal/CentroidalManagerPreviewControlZmp.cpp
  centroidal/CentroidalManagerDdpZmp.cpp
  centroidal/CentroidalManagerFootGuidedControl.cpp
  centroidal/CentroidalManagerIntrinsicallyStableMpc.cpp
  swing/SwingTrajCubicSplineSimple.cpp
  swing/SwingTrajIndHorizontalVertical.cpp
  swing/SwingTrajVariableTaskGain.cpp
  swing/SwingTrajLandingSearch.cpp
  State.cpp
  )

target_include_directories(${CONTROLLER_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)

target_link_libraries(${CONTROLLER_NAME} PUBLIC 
  trajectory_collection::TrajColl
  force_control_collection::ForceColl
  centroidal_control_collection::CCC
)

install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR} EXPORT ${TARGETS_EXPORT_NAME})

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)

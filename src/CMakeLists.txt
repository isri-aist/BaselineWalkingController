set(CONTROLLER_NAME BaselineWalkingController)

add_library(${CONTROLLER_NAME} SHARED
  BaselineWalkingController.cpp
  FootTypes.cpp
  FootManager.cpp
  CentroidalManager.cpp
  centroidal/CentroidalManagerPreviewControlZmp.cpp
  centroidal/CentroidalManagerDdpZmp.cpp
  wrench/Contact.cpp
  wrench/WrenchDistribution.cpp
  trajectory/CubicHermiteSpline.cpp
  trajectory/CubicSpline.cpp
  tasks/FirstOrderImpedanceTask.cpp
  State.cpp
  )
target_link_libraries(${CONTROLLER_NAME} PUBLIC
  mc_rtc::mc_rtc_utils
  mc_rtc::mc_rbdyn
  mc_rtc::mc_control_fsm
  mc_rtc::mc_rtc_ros
  ${catkin_LIBRARIES}
  )

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)

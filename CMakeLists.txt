cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME baseline_walking_controller)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/BaselineWalkingController)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
option(USE_ROS2 "Use ROS2" OFF)

include(cmake/base.cmake)
project(baseline_walking_controller)

# mc_rtc
find_package(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

add_project_dependency(trajectory_collection REQUIRED)
add_project_dependency(force_control_collection REQUIRED)
add_project_dependency(centroidal_control_collection REQUIRED)

if(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
else()
  set(BUILD_TESTING OFF)
endif()

set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})

set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/BaselineWalkingController.yaml")
configure_file(${PROJECT_SOURCE_DIR}/etc/BaselineWalkingController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/BaselineWalkingController DESTINATION include)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

set(PACKAGE_EXTRA_MACROS "set(BWC_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/baseline_walking_controller/states\")
set(BWC_STATES_FILES \"\${BWC_STATES_LIBRARIES}/data\")")

if(USE_ROS2)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  ament_export_dependencies(rosidl_default_runtime)
  ament_export_targets(${CONTROLLER_NAME})
  ament_package()
endif()

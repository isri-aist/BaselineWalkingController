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

project(baseline_walking_controller)

# mc_rtc
find_package(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

if(USE_ROS2)
  set(AMENT_CMAKE_UNINSTALL_TARGET OFF)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(trajectory_collection REQUIRED)
  find_package(force_control_collection REQUIRED)
  find_package(centroidal_control_collection REQUIRED)

  find_package(qp_solver_collection REQUIRED)
  find_package(nmpc_ddp REQUIRED)

  # Eigen
  find_package(Eigen3 REQUIRED)

  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/BaselineWalkingController.yaml")
  set(CONFIG_DIR_STANDALONE "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/BaselineWalkingController")
  set(CONFIG_DIR_CATKIN "${INSTALL_PREFIX_PATH}/lib/mc_controller/BaselineWalkingController")
  if(EXISTS "${CONFIG_DIR_STANDALONE}" AND IS_DIRECTORY "${CONFIG_DIR_STANDALONE}")
    execute_process(COMMAND cmake -E create_symlink
      "${CONFIG_DIR_STANDALONE}" "${CONFIG_DIR_CATKIN}"
    )
  endif()
  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
  set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
else()
  set(BUILD_TESTING OFF)

  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  find_package(trajectory_collection REQUIRED)
  find_package(force_control_collection REQUIRED)
  find_package(centroidal_control_collection REQUIRED)
  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/BaselineWalkingController.yaml")
  set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
endif()

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
  ament_export_targets()
  ament_package()
endif()
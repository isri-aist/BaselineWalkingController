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

include(cmake/base.cmake)
project(baseline_walking_controller LANGUAGES CXX)

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    trajectory_collection
    force_control_collection
    centroidal_control_collection
  )

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})

  catkin_package(
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    trajectory_collection
    force_control_collection
    centroidal_control_collection
    DEPENDS EIGEN3
    INCLUDE_DIRS include
    LIBRARIES BaselineWalkingController
  )

  include_directories(include ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})
  set(CONFIG_OUT "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/BaselineWalkingController.yaml")
  set(CONFIG_DIR_STANDALONE "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/BaselineWalkingController")
  set(CONFIG_DIR_CATKIN "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/BaselineWalkingController")
  if(EXISTS "${CONFIG_DIR_STANDALONE}" AND IS_DIRECTORY "${CONFIG_DIR_STANDALONE}")
    execute_process(COMMAND cmake -E create_symlink
      "${CONFIG_DIR_STANDALONE}" "${CONFIG_DIR_CATKIN}"
    )
  endif()
  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
  set(STATES_INSTALL_PREFIX ${CATKIN_DEVEL_PREFIX}/lib)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(CATKIN_ENABLE_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  add_project_dependency(trajectory_collection REQUIRED)
  add_project_dependency(force_control_collection REQUIRED)
  add_project_dependency(centroidal_control_collection REQUIRED)
  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/BaselineWalkingController.yaml")
  set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
endif()

configure_file(etc/BaselineWalkingController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/BaselineWalkingController DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")

if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

set(PACKAGE_EXTRA_MACROS "set(BWC_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/baseline_walking_controller/states\")
set(BWC_STATES_FILES \"\${BWC_STATES_LIBRARIES}/data\")")

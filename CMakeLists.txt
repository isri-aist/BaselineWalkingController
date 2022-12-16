cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME baseline_walking_controller)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/BaselineWalkingController)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(baseline_walking_controller LANGUAGES CXX)

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    qp_solver_collection
    centroidal_control_collection
  )

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})

  catkin_package(
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    qp_solver_collection
    centroidal_control_collection
    DEPENDS EIGEN3
    INCLUDE_DIRS include
    LIBRARIES BaselineWalkingController
  )

  include_directories(include ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})
  set(CONFIG_OUT "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/BaselineWalkingController.yaml")
  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(CATKIN_ENABLE_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  add_project_dependency(centroidal_control_collection REQUIRED)
  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/BaselineWalkingController.yaml")
endif()

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

configure_file(etc/BaselineWalkingController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

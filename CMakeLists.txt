cmake_minimum_required(VERSION 3.1)
project(baseline_walking_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  qp_solver_collection
  centroidal_control_collection
  )

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# SpaceVecAlg
find_package(SpaceVecAlg REQUIRED)

# RBDyn
find_package(RBDyn REQUIRED)

# mc_rtc
find_package(mc_rtc REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  qp_solver_collection
  centroidal_control_collection
  DEPENDS EIGEN3
  INCLUDE_DIRS include
  LIBRARIES BaselineWalkingController
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

configure_file(etc/BaselineWalkingController.in.yaml
  "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/BaselineWalkingController.yaml" @ONLY)

add_subdirectory(src)

OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

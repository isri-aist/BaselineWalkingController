<<<<<<< HEAD
add_subdirectory(controller)
add_subdirectory(SwingTraj)
=======
find_package(GTest REQUIRED)
include(GoogleTest)
# Prevents discovery failure before install
# No effect in CMake < 3.18
set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)

function(add_BWC_test NAME)
  add_executable(${NAME} src/${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC GTest::gtest BaselineWalkingController mc_rtc::mc_rtc_utils)
  gtest_discover_tests(${NAME})
endfunction()


set(BWC_gtest_list
  TestSwingTraj
  )

foreach(NAME IN LISTS BWC_gtest_list)
  add_BWC_test(${NAME})
endforeach()
>>>>>>> 361233b (feat(test): remove ros dependencies)

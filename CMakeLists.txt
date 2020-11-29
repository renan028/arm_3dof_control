cmake_minimum_required(VERSION 3.2)
project(RemyRobotControl)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTest)

include_directories(
  include/remy_robot_control
)
add_executable(${PROJECT_NAME} src/main.cc 
  src/control.cc 
  src/robot.cc 
  src/utils.cc
  src/robot_system.cc)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen -pthread)

if(GTEST_FOUND)
  get_filename_component(DATA_TEST_DIR "tests/data" ABSOLUTE)
  configure_file(tests/settings.h.in tests/settings.h)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/tests)
  enable_testing()
  add_executable(${PROJECT_NAME}_Test tests/test_main.cc src/robot.cc src/utils.cc) 
  add_executable(${PROJECT_NAME}_Control_Test tests/test_control.cc 
      src/control.cc 
      src/robot.cc 
      src/utils.cc) 
  add_executable(${PROJECT_NAME}_Test_Integration tests/test_integration.cc 
      src/control.cc 
      src/robot.cc 
      src/utils.cc
      src/robot_system.cc) 
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}_Test GTest::GTest GTest::Main Eigen3::Eigen)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}_Control_Test GTest::GTest GTest::Main Eigen3::Eigen)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME}_Test_Integration GTest::GTest GTest::Main Eigen3::Eigen)
endif()
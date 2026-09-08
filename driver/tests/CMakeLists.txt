cmake_minimum_required(VERSION 3.22)
project(sg200x_clock_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../sgll" sgll)

add_library(clock_controller STATIC ../sg200x_rcc.cpp)
target_include_directories(clock_controller PUBLIC .. ../../libxr/src/core)
target_link_libraries(clock_controller PUBLIC sgll)

add_executable(clock_planner_test clock_planner_test.cpp)
target_link_libraries(clock_planner_test PRIVATE clock_controller)
add_executable(clock_controller_test clock_controller_test.cpp)
target_link_libraries(clock_controller_test PRIVATE clock_controller)

enable_testing()
add_test(NAME clock_planner COMMAND clock_planner_test)
add_test(NAME clock_controller COMMAND clock_controller_test)

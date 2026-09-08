cmake_minimum_required(VERSION 3.22)
project(sg200x_application_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../sgll" sgll)

add_executable(camera_mailbox_test camera_mailbox_test.cpp ../camera_mailbox.cpp)
target_include_directories(camera_mailbox_test PRIVATE ..)
target_link_libraries(camera_mailbox_test PRIVATE sgll)

enable_testing()
add_test(NAME camera_mailbox COMMAND camera_mailbox_test)

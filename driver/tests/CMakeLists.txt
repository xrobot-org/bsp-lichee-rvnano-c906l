cmake_minimum_required(VERSION 3.22)
project(sg200x_clock_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../sg200x-ll-driver" sg200x_ll)

add_library(clock_controller STATIC ../sg200x_rcc.cpp)
target_include_directories(clock_controller PUBLIC .. ../../libxr/src/core)
target_link_libraries(clock_controller PUBLIC sg200x_ll)

add_executable(clock_planner_test clock_planner_test.cpp)
target_link_libraries(clock_planner_test PRIVATE clock_controller)
add_executable(clock_controller_test clock_controller_test.cpp)
target_link_libraries(clock_controller_test PRIVATE clock_controller)

enable_testing()
add_test(NAME clock_planner COMMAND clock_planner_test)
add_test(NAME clock_controller COMMAND clock_controller_test)

# Use the actual LibXR interfaces and host synchronization backend. Only CSR
# cache operations and I2C enable acknowledgment need test hardware hooks.
set(LIBXR_SYSTEM linux)
set(LIBXR_DRIVER "")
set(LIBXR_NO_EIGEN ON)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../libxr" libxr EXCLUDE_FROM_ALL)
# LibXR's Linux system backend requires its monotonic clock; the Linux USB,
# networking, and device drivers are unnecessary for these register-image tests.
target_sources(xr PRIVATE ../../libxr/driver/linux/linux_timebase.cpp)
target_include_directories(xr PRIVATE ../../libxr/driver/linux)
add_executable(peripheral_driver_test
  peripheral_driver_test.cpp
  ../sg200x_gpio.cpp
  ../sg200x_pwm.cpp
  ../sg200x_adc.cpp
  ../sg200x_watchdog.cpp
  ../sg200x_i2c.cpp
  ../sg200x_spi.cpp
)
target_compile_options(peripheral_driver_test PRIVATE
  -include "${CMAKE_CURRENT_SOURCE_DIR}/host_core.hpp")
target_link_options(peripheral_driver_test PRIVATE -Wl,--wrap=sg200x_ll_i2c_enable_wait)
target_link_libraries(peripheral_driver_test PRIVATE clock_controller xr)
add_test(NAME peripheral_drivers COMMAND peripheral_driver_test)

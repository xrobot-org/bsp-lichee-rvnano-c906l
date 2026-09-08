# This file is staged into the SG200x SDK by scripts/sg200x-sdk/prepare.sh.
if(NOT DEFINED ENV{SG200X_BSP_ROOT})
  message(FATAL_ERROR "SG200X_BSP_ROOT is required for the XRobot SDK overlay.")
endif()

set(SG200X_BSP_ROOT "$ENV{SG200X_BSP_ROOT}")

enable_language(CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIBXR_SYSTEM freertos)
set(LIBXR_DRIVER "")
set(LIBXR_SINGLE_CORE ON)
set(LIBXR_STATIC_BUILD ON)
set(LIBXR_NO_EIGEN ON)
set(LIBXR_PRINT_ENABLE_FLOAT 0)
add_subdirectory("${SG200X_BSP_ROOT}/libxr" "${CMAKE_BINARY_DIR}/xrobot-libxr")

set(_xrobot_sdk_include_dirs
  "${TOP_DIR}/install/include/kernel"
  "${TOP_DIR}/install/include/arch"
  "${TOP_DIR}/install/include/common"
)
target_include_directories(xr PUBLIC ${_xrobot_sdk_include_dirs})
target_compile_definitions(xr PUBLIC
  THEAD_C906
)

add_library(xrobot STATIC
  "${SG200X_BSP_ROOT}/driver/sg200x_timebase.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_rcc.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_gpio.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_pwm.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_adc.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_watchdog.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_i2c.cpp"
  "${SG200X_BSP_ROOT}/driver/sg200x_spi.cpp"
)

# Applications can own their source list without changing the BSP overlay.
if(EXISTS "${SG200X_BSP_ROOT}/User/CMakeLists.txt")
  add_subdirectory("${SG200X_BSP_ROOT}/User" "${CMAKE_BINARY_DIR}/xrobot-user")
else()
  target_sources(xrobot PRIVATE
    "${SG200X_BSP_ROOT}/User/main.cpp"
    "${SG200X_BSP_ROOT}/User/platform.cpp"
  )
endif()

add_subdirectory("${SG200X_BSP_ROOT}/sgll" sgll)
target_include_directories(sgll SYSTEM PUBLIC "${TOP_DIR}/install/include/arch")

target_include_directories(xrobot PRIVATE
  ${_xrobot_sdk_include_dirs}
  "${SG200X_BSP_ROOT}/User"
  "${SG200X_BSP_ROOT}/driver"
)
target_compile_options(xrobot PRIVATE -fno-exceptions -fno-rtti)
target_compile_options(xr PRIVATE -fno-exceptions -fno-rtti)
target_compile_definitions(xrobot PRIVATE
  THEAD_C906
)
target_link_libraries(xrobot PUBLIC xr kernel stdc++ sgll)

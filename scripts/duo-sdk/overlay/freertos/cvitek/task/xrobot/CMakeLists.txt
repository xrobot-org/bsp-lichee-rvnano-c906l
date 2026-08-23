# This file is staged into the Vendor SDK by scripts/duo-sdk/prepare.sh.
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
  "${SG200X_BSP_ROOT}/User/main.cpp"
  "${SG200X_BSP_ROOT}/sg200x-c906l-xr-driver/sg200x_timebase.cpp"
  "${SG200X_BSP_ROOT}/sg200x-c906l-xr-driver/sg200x_gpio.cpp"
  "${SG200X_BSP_ROOT}/sg200x-c906l-xr-driver/sg200x_pwm.cpp"
)

target_include_directories(xrobot PRIVATE
  ${_xrobot_sdk_include_dirs}
  "${SG200X_BSP_ROOT}/User"
  "${SG200X_BSP_ROOT}/sg200x-c906l-xr-driver"
)
target_compile_options(xrobot PRIVATE -fno-exceptions -fno-rtti)
target_compile_options(xr PRIVATE -fno-exceptions -fno-rtti)
target_compile_definitions(xrobot PRIVATE
  THEAD_C906
)
if(DEFINED ENV{SG200X_TIMEBASE_TEST})
  target_compile_definitions(xrobot PRIVATE SG200X_TIMEBASE_TEST=1)
endif()
if(DEFINED ENV{SG200X_PREEMPT_TEST})
  target_compile_definitions(xrobot PRIVATE SG200X_PREEMPT_TEST=1)
endif()
target_link_libraries(xrobot PUBLIC xr kernel stdc++)

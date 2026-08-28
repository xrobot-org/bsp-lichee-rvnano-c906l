cmake_minimum_required(VERSION 3.22)

project(sg200x_c906_bsp LANGUAGES NONE)

set(SG200X_SDK_DIR "$ENV{SG200X_SDK_DIR}" CACHE PATH
    "Clean SG200x C906L SDK worktree at the pinned commit")
set(SG200X_RUNTIME_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/build/firmware" CACHE PATH
    "Directory for the runtime-replaceable C906 firmware")
set(SG200X_BASH "$ENV{SG200X_BASH}" CACHE FILEPATH
    "Native Bash executable used to invoke the Vendor SDK build scripts")
if(NOT SG200X_BASH)
  set(SG200X_BASH "bash" CACHE FILEPATH
      "Native Bash executable used to invoke the Vendor SDK build scripts" FORCE)
endif()
set(SG200X_CROSS_COMPILE "$ENV{SG200X_CROSS_COMPILE}" CACHE STRING
    "RISC-V GNU compiler prefix, including the trailing hyphen")
set(SG200X_TOOL_SUFFIX "$ENV{SG200X_TOOL_SUFFIX}" CACHE STRING
    "Optional executable suffix for the cross compiler")
if(WIN32 AND NOT SG200X_TOOL_SUFFIX)
  set(SG200X_TOOL_SUFFIX ".exe" CACHE STRING
      "Optional executable suffix for the cross compiler" FORCE)
endif()
set(SG200X_TARGET "$ENV{SG200X_TARGET}" CACHE STRING
    "SSH target in the form user@board-ip")
set(SG200X_SSH_PORT "$ENV{SG200X_SSH_PORT}" CACHE STRING
    "Optional SSH port used by the deploy target")
set(SG200X_BOARD_DEPLOY_CONFIG
    "${CMAKE_SOURCE_DIR}/sdk/sg200x/licheervnano/deploy.conf" CACHE FILEPATH
    "Board ownership configuration applied before C906L starts")

if(NOT SG200X_SDK_DIR)
  message(FATAL_ERROR
    "Set SG200X_SDK_DIR to a native filesystem SG200x SDK worktree before configuring.")
endif()
if(WIN32 AND SG200X_SDK_DIR MATCHES "^[\\/]{2}")
  message(FATAL_ERROR
    "SG200X_SDK_DIR must be a local Windows path; UNC and virtual filesystem paths are unsupported.")
endif()
if(NOT EXISTS "${SG200X_SDK_DIR}/freertos/cvitek/build_cv181x.sh")
  message(FATAL_ERROR
    "SG200X_SDK_DIR is not a complete native SG200x SDK worktree: ${SG200X_SDK_DIR}")
endif()
if(WIN32 AND NOT SG200X_CROSS_COMPILE)
  message(FATAL_ERROR
    "Set SG200X_CROSS_COMPILE to the Windows xPack riscv-none-elf- compiler prefix.")
endif()
if(IS_ABSOLUTE "${SG200X_BASH}" AND NOT EXISTS "${SG200X_BASH}")
  message(FATAL_ERROR "SG200X_BASH does not exist: ${SG200X_BASH}")
endif()
if(WIN32 AND SG200X_BASH MATCHES "[\\\\/]Windows[\\\\/]System32[\\\\/]bash\\.exe$")
  message(FATAL_ERROR
    "SG200X_BASH points to the system Linux launcher; use Scoop Git Bash instead.")
endif()

set(_firmware_command
  ${CMAKE_COMMAND} -E env
    "SG200X_SDK_DIR=${SG200X_SDK_DIR}"
    "SG200X_CROSS_COMPILE=${SG200X_CROSS_COMPILE}"
    "SG200X_TOOL_SUFFIX=${SG200X_TOOL_SUFFIX}"
    "OUTPUT_DIR=${SG200X_RUNTIME_OUTPUT_DIR}"
    "${SG200X_BASH}" -lc
    "exec '${CMAKE_SOURCE_DIR}/scripts/sg200x-sdk/build.sh'"
)

add_custom_target(firmware
  COMMAND ${_firmware_command}
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
)

add_custom_target(deploy
  COMMAND ${CMAKE_COMMAND} -E env
    "SG200X_TARGET=${SG200X_TARGET}"
    "SG200X_SSH_PORT=${SG200X_SSH_PORT}"
    "SG200X_BOARD_CONFIG=${SG200X_BOARD_DEPLOY_CONFIG}"
    "${SG200X_BASH}" -lc
    "exec '${CMAKE_SOURCE_DIR}/scripts/deploy_rtos_fw.sh' '${SG200X_RUNTIME_OUTPUT_DIR}/c906-mcu.elf'"
  DEPENDS firmware
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL
)

if(WIN32)
  message(STATUS "Bash: ${SG200X_BASH}")
  message(STATUS "RISC-V compiler prefix: ${SG200X_CROSS_COMPILE}")
endif()

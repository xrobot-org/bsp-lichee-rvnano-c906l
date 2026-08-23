cmake_minimum_required(VERSION 3.22)

project(sg200x_c906_bsp LANGUAGES NONE)

set(SG200X_DUO_SDK_DIR "" CACHE PATH
    "Clean Duo SDK worktree at the pinned commit")
set(SG200X_RUNTIME_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/build/firmware" CACHE PATH
    "Directory for the runtime-replaceable C906 firmware")
set(SG200X_BASH "bash" CACHE STRING
    "Bash executable used to invoke the Vendor SDK build scripts")
set(SG200X_WSL_DISTRO "" CACHE STRING
    "WSL distribution that runs the Vendor SDK scripts on Windows")
set(SG200X_CROSS_COMPILE "" CACHE STRING
    "RISC-V GNU compiler prefix, including the trailing hyphen")
set(SG200X_WCH_TOOLCHAIN_DIR "" CACHE PATH
    "Directory containing Windows WCH RISC-V GNU tool executables")
set(SG200X_CLANGD_SDK_DIR "" CACHE STRING
    "SDK path used in Windows clangd compilation databases")
set(SG200X_CLANGD_CROSS_COMPILE "" CACHE STRING
    "Compiler prefix used in Windows clangd compilation databases")

if(WIN32 AND NOT SG200X_DUO_SDK_DIR)
  message(FATAL_ERROR
    "Set the Windows SG200X_DUO_SDK_DIR environment variable to the sparse Duo SDK worktree before configuring.")
endif()

if(SG200X_DUO_SDK_DIR)
  set(_firmware_command
    ${CMAKE_COMMAND} -E env
      "SG200X_DUO_SDK_DIR=${SG200X_DUO_SDK_DIR}"
      "SG200X_CROSS_COMPILE=${SG200X_CROSS_COMPILE}"
      "OUTPUT_DIR=${SG200X_RUNTIME_OUTPUT_DIR}"
      "${SG200X_BASH}" "${CMAKE_SOURCE_DIR}/scripts/duo-sdk/build.sh"
  )

  if(WIN32 AND SG200X_WSL_DISTRO)
    # The SDK's Python and shell tooling requires POSIX paths. The compiler
    # prefix may still point at a Windows-hosted .exe through /mnt/c.
    file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}/scripts/duo-sdk/build.sh" _script_path)
    file(TO_CMAKE_PATH "${SG200X_RUNTIME_OUTPUT_DIR}" _output_path)
    foreach(_path_var IN ITEMS _script_path _output_path)
      string(REGEX MATCH "^([A-Za-z]):/(.*)" _match "${${_path_var}}")
      if(NOT _match)
        message(FATAL_ERROR "Cannot convert ${${_path_var}} to a WSL path.")
      endif()
      string(TOLOWER "${CMAKE_MATCH_1}" _drive)
      set(${_path_var} "/mnt/${_drive}/${CMAKE_MATCH_2}")
    endforeach()
    set(_firmware_command
      "${SG200X_BASH}" --distribution "${SG200X_WSL_DISTRO}" -- env
        "SG200X_DUO_SDK_DIR=${SG200X_DUO_SDK_DIR}"
        "SG200X_CROSS_COMPILE=${SG200X_CROSS_COMPILE}"
        "SG200X_WCH_TOOLCHAIN_DIR=${SG200X_WCH_TOOLCHAIN_DIR}"
        "OUTPUT_DIR=${_output_path}"
        "CLANGD_OUTPUT_DIR=${_output_path}/../clangd"
        "SG200X_CLANGD_SDK_DIR=${SG200X_CLANGD_SDK_DIR}"
        "SG200X_CLANGD_CROSS_COMPILE=${SG200X_CLANGD_CROSS_COMPILE}"
        bash "${_script_path}"
    )
  endif()

  add_custom_target(firmware
    COMMAND ${_firmware_command}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    USES_TERMINAL
  )
  add_custom_target(deploy
    COMMAND "${SG200X_BASH}" "${CMAKE_SOURCE_DIR}/scripts/deploy_rtos_fw.sh"
      "${SG200X_RUNTIME_OUTPUT_DIR}/c906-mcu.elf"
    DEPENDS firmware
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    USES_TERMINAL
  )
else()
  add_custom_target(firmware
    COMMAND ${CMAKE_COMMAND} -E echo
      "Configure with -DSG200X_DUO_SDK_DIR=/path/to/duo-sdk-c906"
  )
  add_custom_target(deploy
    COMMAND ${CMAKE_COMMAND} -E echo
      "Configure with -DSG200X_DUO_SDK_DIR before deploying firmware"
  )
endif()

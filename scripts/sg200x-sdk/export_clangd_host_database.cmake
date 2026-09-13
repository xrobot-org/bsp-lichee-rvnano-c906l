# Export the clangd compilation database for the host-side sources.
#
# The firmware database (build/clangd) only describes translation units that are
# cross-compiled for the C906. Host-only code -- driver/tests, sg200x-ll-driver/tests, and
# tools -- is compiled by native GCC for Linux, so a firmware command cannot
# parse it: the RISC-V newlib sysroot has no POSIX headers, and sys/mman.h and
# ucontext.h are reported as missing.
#
# This configures the standalone host CMake projects with
# CMAKE_EXPORT_COMPILE_COMMANDS=ON, merges their databases into one, and appends
# entries for the host sources that no CMake project owns. The root .clangd
# sends the affected paths here.
#
# Run it through the `clangd` build target, or directly:
#
#   cmake -DBSP_ROOT=<bsp> -P scripts/sg200x-sdk/export_clangd_host_database.cmake
#
# Missing optional pieces are reported as warnings rather than failures, so a
# firmware build that only wants the database never breaks on a host that cannot
# configure the test projects.

cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED BSP_ROOT OR BSP_ROOT STREQUAL "")
  message(FATAL_ERROR "BSP_ROOT is required")
endif()
get_filename_component(bsp_root "${BSP_ROOT}" ABSOLUTE)

if(DEFINED OUTPUT_DIR AND NOT OUTPUT_DIR STREQUAL "")
  get_filename_component(output_dir "${OUTPUT_DIR}" ABSOLUTE)
else()
  set(output_dir "${bsp_root}/build/clangd-host")
endif()

if(DEFINED FIRMWARE_DATABASE AND NOT FIRMWARE_DATABASE STREQUAL "")
  get_filename_component(firmware_database "${FIRMWARE_DATABASE}" ABSOLUTE)
else()
  set(firmware_database "${bsp_root}/build/clangd/compile_commands.json")
endif()

find_program(ninja_program NAMES ninja)
find_program(host_cc NAMES gcc cc clang)
find_program(riscv_linux_cc NAMES riscv64-linux-gnu-gcc)

if(NOT host_cc)
  message(FATAL_ERROR "a host C compiler (gcc) is required for the host database")
endif()

# JSON string escaping for the entries this script writes itself.
function(escape_json output value)
  string(REPLACE "\\" "\\\\" escaped "${value}")
  string(REPLACE "\"" "\\\"" escaped "${escaped}")
  set(${output} "${escaped}" PARENT_SCOPE)
endfunction()

# Append one hand-written entry. Every chunk is terminated by ",\n" so the caller
# can trim the final separator once at the end.
function(append_entry accumulator directory source command)
  escape_json(directory_json "${directory}")
  escape_json(source_json "${source}")
  escape_json(command_json "${command}")
  # string(CONCAT) rather than multi-argument set(): a multi-argument set()
  # builds a CMake list and would inject ";" separators into the JSON.
  string(CONCAT chunk
    "  {\n    \"directory\": \"${directory_json}\",\n"
    "    \"file\": \"${source_json}\",\n"
    "    \"command\": \"${command_json}\"\n  },\n")
  set(${accumulator} "${${accumulator}}${chunk}" PARENT_SCOPE)
endfunction()

# Append every element of a compile_commands.json array, skipping translation
# units an earlier database already described: both host projects compile the
# SG200X LL sources, and clangd should see one entry per file. The set of seen paths
# is newline-delimited because paths cannot contain newlines, while a CMake list
# would break on the ";" that legal paths may contain.
function(append_database accumulator seen_var database)
  if(NOT EXISTS "${database}")
    return()
  endif()
  file(READ "${database}" content)
  string(STRIP "${content}" content)
  if(content STREQUAL "" OR content STREQUAL "[]")
    return()
  endif()
  string(JSON count LENGTH "${content}")
  if(count EQUAL 0)
    return()
  endif()
  set(result "${${accumulator}}")
  set(seen "${${seen_var}}")
  math(EXPR last "${count} - 1")
  foreach(index RANGE 0 ${last})
    string(JSON source GET "${content}" ${index} file)
    string(FIND "${seen}" "\n${source}\n" duplicate)
    if(NOT duplicate EQUAL -1)
      continue()
    endif()
    string(JSON element GET "${content}" ${index})
    set(result "${result}${element},\n")
    set(seen "${seen}${source}\n")
  endforeach()
  set(${accumulator} "${result}" PARENT_SCOPE)
  set(${seen_var} "${seen}" PARENT_SCOPE)
endfunction()

# The accumulator holds raw JSON, so it must never be treated as a CMake list:
# every read and write of it stays quoted, and list operations are avoided.
set(entries "")
set(seen_files "\n")

# ---------------------------------------------------------------------------
# Host CMake projects
# ---------------------------------------------------------------------------
set(configured 0)
set(generator_args "")
if(ninja_program)
  set(generator_args -G Ninja)
endif()

foreach(project IN ITEMS driver/tests sg200x-ll-driver/tests)
  set(source "${bsp_root}/${project}")
  if(NOT EXISTS "${source}/CMakeLists.txt")
    message(WARNING "skipping ${project}: no CMakeLists.txt")
    continue()
  endif()
  # Both projects are named "tests", so key the build directory on the full
  # relative path.
  string(REPLACE "/" "-" project_slug "${project}")
  set(build "${output_dir}/${project_slug}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${source}" -B "${build}" ${generator_args}
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    WORKING_DIRECTORY "${bsp_root}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE configure_stdout
    ERROR_VARIABLE configure_stderr)
  if(NOT result EQUAL 0)
    message(WARNING "could not configure ${project}; its entries are missing:\n"
                    "${configure_stderr}")
    continue()
  endif()
  append_database(entries seen_files "${build}/compile_commands.json")
  math(EXPR configured "${configured} + 1")
endforeach()

# ---------------------------------------------------------------------------
# Host sources no CMake project compiles
# ---------------------------------------------------------------------------
# run.py builds the sg200x-ll cache probes with ad-hoc compiler invocations, and the
# on-board tools are built by their own shell scripts, so clangd has nothing to
# fall back on for either.
foreach(relative IN ITEMS
    tools/sc035hgs-reg/sc035hgs-reg.c
    tools/sg2002-camera-bridge/bridge.c
    tools/sg2002-camera-bridge/mmfprobe.c
    sg200x-ll-driver/tests/cache_linux.c)
  set(source "${bsp_root}/${relative}")
  if(NOT EXISTS "${source}")
    continue()
  endif()
  set(standard c11)
  set(flags "")
  set(driver "${host_cc}")
  if(relative STREQUAL "sg200x-ll-driver/tests/cache_linux.c")
    # This file records a RISC-V signal frame, so its mcontext_t and REG_PC only
    # exist in the RISC-V Linux sysroot.
    set(standard c23)
    set(flags "-I${bsp_root}/sg200x-ll-driver/inc;-I${bsp_root}/sg200x-ll-driver/device")
    if(riscv_linux_cc)
      set(driver "${riscv_linux_cc}")
    else()
      message(WARNING "riscv64-linux-gnu-gcc not found; using ${host_cc} for "
                      "${relative}, whose mcontext_t layout will not match")
    endif()
  elseif(relative STREQUAL "tools/sg2002-camera-bridge/bridge.c")
    set(flags "-D_DEFAULT_SOURCE")
  endif()
  set(command "${driver} -Wall -Wextra -std=${standard}")
  separate_arguments(flags_list UNIX_COMMAND "${flags}")
  foreach(flag IN LISTS flags_list)
    set(command "${command} ${flag}")
  endforeach()
  set(command "${command} -c ${source}")
  get_filename_component(source_dir "${source}" DIRECTORY)
  append_entry(entries "${source_dir}" "${source}" "${command}")
endforeach()

# cache_probe.c is built for the C906 only. A host command would hide the cache
# intrinsics behind their __riscv guards, so borrow the flags of a firmware
# translation unit that includes the same SG200X LL and SDK headers.
set(cross_source "${bsp_root}/sg200x-ll-driver/tests/cache_probe.c")
set(cross_donor "${bsp_root}/sg200x-ll-driver/src/sg200x_ll_csr.c")
if(EXISTS "${cross_source}")
  if(NOT EXISTS "${firmware_database}")
    message(WARNING "no firmware database at ${firmware_database}; "
                    "sg200x-ll-driver/tests/cache_probe.c will fall back to inference")
  else()
    file(READ "${firmware_database}" firmware)
    string(JSON firmware_count LENGTH "${firmware}")
    set(donor_index -1)
    if(firmware_count GREATER 0)
      math(EXPR last "${firmware_count} - 1")
      foreach(index RANGE 0 ${last})
        string(JSON candidate GET "${firmware}" ${index} file)
        if(candidate STREQUAL cross_donor)
          set(donor_index ${index})
          break()
        endif()
      endforeach()
    endif()
    if(donor_index LESS 0)
      message(WARNING "no firmware entry for ${cross_donor}; "
                      "sg200x-ll-driver/tests/cache_probe.c will fall back to inference")
    else()
      string(JSON donor_directory GET "${firmware}" ${donor_index} directory)
      string(JSON command GET "${firmware}" ${donor_index} command)
      string(REGEX REPLACE " -o +[^ ]+" "" command "${command}")
      string(REPLACE "${cross_donor}" "${cross_source}" command "${command}")
      append_entry(entries "${donor_directory}" "${cross_source}" "${command}")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# Write the database
# ---------------------------------------------------------------------------
if(configured EQUAL 0 AND entries STREQUAL "")
  message(FATAL_ERROR "no host entries were produced; is this a BSP checkout?")
endif()

# Drop the trailing ",\n" left by the chunk convention.
string(LENGTH "${entries}" entries_length)
if(entries_length GREATER 1)
  math(EXPR keep "${entries_length} - 2")
  string(SUBSTRING "${entries}" 0 ${keep} entries)
endif()

file(WRITE "${output_dir}/compile_commands.json" "[\n${entries}\n]\n")
message(STATUS "Wrote ${output_dir}/compile_commands.json")

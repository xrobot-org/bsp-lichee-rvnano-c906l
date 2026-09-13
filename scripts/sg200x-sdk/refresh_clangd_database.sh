#!/usr/bin/env bash
# Refresh both clangd databases without rebuilding the firmware.
#
# A firmware build refreshes build/clangd and build/clangd-host on every run, but
# a source restructure (moving the device header, renaming the library directory,
# editing the SDK overlay) makes them stale without touching the compiler. clangd
# then resolves includes against directories that no longer exist and reports the
# library headers as missing.
#
# This runs the same two export steps the firmware build runs, reconfiguring the
# SDK task project first so its native database matches the current sources. It
# touches no compiled output and needs no cross compiler configuration beyond what
# the staged SDK build tree already records.
#
# Usage:
#   scripts/sg200x-sdk/refresh_clangd_database.sh
set -euo pipefail

port_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bsp_root=$(cd "$port_root/../.." && pwd)
cvitek=$bsp_root/build/sdk-worktree/freertos/cvitek
clangd_dir=$bsp_root/build/clangd
clangd_host_dir=$bsp_root/build/clangd-host
task_build=$cvitek/build/task
task_cache=$task_build/CMakeCache.txt
native_database=$task_build/compile_commands.json

refresh_firmware_database() {
  if [[ ! -f $task_cache ]]; then
    echo "clangd: no staged SDK build tree; run a firmware build once to create it." >&2
    return 0
  fi

  # The staged overlay is a snapshot of sdk/sg200x/overlay; keep it current so a
  # renamed or moved library directory reaches the configure step.
  local overlay_source=$bsp_root/sdk/sg200x/overlay/freertos/cvitek/task/xrobot/CMakeLists.txt
  local overlay_staged=$cvitek/task/xrobot/CMakeLists.txt
  if [[ -f $overlay_staged && -f $overlay_source ]] && ! cmp -s "$overlay_source" "$overlay_staged"; then
    cp "$overlay_source" "$overlay_staged"
    echo "clangd: refreshed the staged SDK overlay"
  fi

  # Reuse the cross compiler recorded by the last configure unless the caller
  # overrides the prefix; the SDK toolchain file falls back to a bare-metal
  # prefix that a Linux host usually does not have.
  if [[ -z ${SG200X_CROSS_COMPILE:-} ]]; then
    local compiler prefix
    compiler=$(sed -n 's/^CMAKE_C_COMPILER:FILEPATH=//p' "$task_cache" | head -n 1)
    # Strip the executable suffix record left by a Windows configure (gcc.exe).
    prefix=${compiler%.exe}
    if [[ $prefix == *gcc ]]; then
      export SG200X_CROSS_COMPILE=${prefix%gcc}
      export PATH="${SG200X_CROSS_COMPILE%/*}:$PATH"
    fi
  fi
  export SG200X_BSP_ROOT=$bsp_root
  export DDR_64MB_SIZE=${DDR_64MB_SIZE:-n}

  # Configure only: CMake writes compile_commands.json without compiling, and the
  # arguments mirror build_cv181x.sh so the database keeps the firmware flags.
  cmake -G Ninja -DCHIP=cv181x -DRUN_ARCH=riscv64 -DRUN_TYPE=CVIRTOS \
    -DTOP_DIR="$cvitek" -DBUILD_ENV_PATH="$cvitek/sg200x-licheervnano-env" \
    -DBOARD_FPGA=n -DCMAKE_TOOLCHAIN_FILE="$cvitek/scripts/toolchain-riscv64-elf.cmake" \
    -S "$cvitek/task" -B "$task_build"

  if [[ -s $native_database ]]; then
    mkdir -p "$clangd_dir"
    python3 "$port_root/export_clangd_database.py" "$native_database" \
      "$clangd_dir/compile_commands.json"
    echo "clangd: refreshed build/clangd/compile_commands.json"
  fi
}

refresh_firmware_database

# The host database merges the firmware entries, so it runs after the refresh
# above. Missing host projects are warnings, matching the firmware build.
cmake -DBSP_ROOT="$bsp_root" -DOUTPUT_DIR="$clangd_host_dir" \
  -P "$port_root/export_clangd_host_database.cmake"
echo "clangd: refreshed build/clangd-host/compile_commands.json"

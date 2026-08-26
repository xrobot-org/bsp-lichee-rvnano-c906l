#!/usr/bin/env bash
# Stage BSP-owned files into a dedicated pinned SDK worktree.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: prepare.sh <sg200x-sdk-worktree>" >&2
  exit 2
fi

port_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${SG200X_BSP_ROOT:?build.sh must set SG200X_BSP_ROOT}"
sdk_assets=$SG200X_BSP_ROOT/sdk/sg200x
sdk_dir=$(cd "$1" && pwd)
build_root=$(cd "$SG200X_BSP_ROOT/build" && pwd)
case "$sdk_dir" in
  "$build_root"/*) ;;
  *)
    echo "The SDK staging directory must be inside the firmware build directory: $build_root" >&2
    exit 1
    ;;
esac
patch_file=$sdk_assets/patches/0001-add-xrobot-task.patch
toolchain_patch_file=$sdk_assets/patches/0003-windows-toolchain.patch
runtime_patch_file=$sdk_assets/patches/0002-run-cxx-constructors.patch

test -d "$sdk_dir/.git" || {
  echo "SDK staging directory is not an initialized disposable worktree: $sdk_dir" >&2
  exit 1
}

stage_dir() {
  local source=$1
  local destination=$2
  test -d "$source" || { echo "Missing overlay directory: $source" >&2; exit 1; }
  mkdir -p "$(dirname "$destination")"
  rm -rf "$destination"
  cp -a "$source" "$destination"
}

test -f "$SG200X_BSP_ROOT/libxr/CMakeLists.txt" || {
  echo "Initialize the libxr submodule before building." >&2
  exit 1
}
test -f "$SG200X_BSP_ROOT/driver/sg200x_timebase.cpp" || {
  echo "SG200x driver sources are missing." >&2
  exit 1
}
test -f "$SG200X_BSP_ROOT/driver/sg200x_pwm.cpp" || {
  echo "SG200x PWM driver source is missing." >&2
  exit 1
}
test -f "$SG200X_BSP_ROOT/driver/sg200x_adc.cpp" || {
  echo "SG200x ADC driver source is missing." >&2
  exit 1
}
test -f "$SG200X_BSP_ROOT/driver/sg200x_mmio.hpp" || {
  echo "SG200x MMIO helper is missing." >&2
  exit 1
}

stage_dir "$sdk_assets/overlay/freertos/cvitek/task/xrobot" \
  "$sdk_dir/freertos/cvitek/task/xrobot"

if git -C "$sdk_dir" apply --check "$patch_file" >/dev/null 2>&1; then
  git -C "$sdk_dir" apply "$patch_file"
elif grep -q 'SG200X_CROSS_COMPILE' "$sdk_dir/freertos/cvitek/scripts/toolchain-riscv64-elf.cmake" \
    && grep -q 'add_subdirectory(xrobot)' "$sdk_dir/freertos/cvitek/task/CMakeLists.txt" \
    && grep -q 'configASSERT(x)' "$sdk_dir/freertos/cvitek/kernel/include/riscv64/FreeRTOSConfig.h"; then
  echo "BSP patch is already present in the SDK worktree."
else
  echo "SDK worktree contains a partial or incompatible BSP patch." >&2
  exit 1
fi

if git -C "$sdk_dir" apply --recount --check "$toolchain_patch_file" >/dev/null 2>&1; then
  git -C "$sdk_dir" apply --recount "$toolchain_patch_file"
elif ! grep -q 'objcopy${SG200X_TOOL_SUFFIX}' "$sdk_dir/freertos/cvitek/scripts/toolchain-riscv64-elf.cmake"; then
  echo "SDK toolchain staging patch is missing or incompatible." >&2
  exit 1
fi

if git -C "$sdk_dir" apply --recount --check "$runtime_patch_file" >/dev/null 2>&1; then
  git -C "$sdk_dir" apply --recount "$runtime_patch_file"
elif grep -q 'sg200x_run_global_constructors' "$sdk_dir/freertos/cvitek/arch/riscv64/src/start.S" \
    && grep -q '__init_array_start' "$sdk_dir/freertos/cvitek/scripts/cv181x_lscript.ld"; then
  echo "C++ constructor runtime patch is already present in the SDK worktree."
else
  echo "SDK C++ constructor runtime patch is missing or incompatible." >&2
  exit 1
fi

#!/usr/bin/env bash
# Stage BSP-owned files into a dedicated pinned SDK worktree.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: prepare.sh <duo-sdk-worktree>" >&2
  exit 2
fi

port_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
sdk_dir=$(cd "$1" && pwd)
patch_file=$port_root/patches/0001-add-xrobot-task.patch

: "${SG200X_BSP_ROOT:?build.sh must set SG200X_BSP_ROOT}"

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
test -f "$SG200X_BSP_ROOT/sg200x-c906l-xr-driver/sg200x_timebase.cpp" || {
  echo "SG200x driver sources are missing." >&2
  exit 1
}
test -f "$SG200X_BSP_ROOT/sg200x-c906l-xr-driver/sg200x_pwm.cpp" || {
  echo "SG200x PWM driver source is missing." >&2
  exit 1
}

# Revert the previous BSP patch when this dedicated SDK worktree is reused.
# This is faster than resetting a WSL-hosted worktree through Windows Git.
if git -C "$sdk_dir" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  git -C "$sdk_dir" apply --reverse "$patch_file"
fi

stage_dir "$port_root/overlay/freertos/cvitek/task/xrobot" \
  "$sdk_dir/freertos/cvitek/task/xrobot"

git -C "$sdk_dir" apply --check "$patch_file"
git -C "$sdk_dir" apply "$patch_file"

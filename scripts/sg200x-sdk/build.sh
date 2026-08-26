#!/usr/bin/env bash
# Build LicheeRV Nano firmware from the pinned SG200x C906L SDK.
set -euo pipefail

port_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bsp_root=$(cd "$port_root/../.." && pwd)
source "$port_root/sdk.env"

sdk_source_dir=${SG200X_SDK_DIR:-}
output_dir=${OUTPUT_DIR:-$bsp_root/build/firmware}
clangd_dir=${CLANGD_OUTPUT_DIR:-$bsp_root/build/clangd}
sdk_dir=$bsp_root/build/sdk-worktree
rtos_env_dir=$sdk_dir/freertos/cvitek/sg200x-licheervnano-env
rtos_project=sg2002_licheervnano

stage_sdk_worktree() {
  test -n "$sdk_source_dir" || {
    echo "Set SG200X_SDK_DIR to a read-only SG200x SDK source worktree." >&2
    exit 2
  }
  sdk_source_dir=$(cd "$sdk_source_dir" && pwd)
  actual_commit=$(git -C "$sdk_source_dir" rev-parse HEAD)
  if [[ $actual_commit != "$RTOS_SDK_COMMIT" ]]; then
    echo "SDK commit is $actual_commit; expected $RTOS_SDK_COMMIT." >&2
    exit 1
  fi

  # Build only from tracked blobs at the pinned commit. This intentionally
  # ignores files physically present in the source SDK worktree.
  rm -rf "$sdk_dir"
  mkdir -p "$sdk_dir/freertos/cvitek" "$sdk_dir/build/scripts"
  git -C "$sdk_source_dir" archive --format=tar "HEAD:freertos/cvitek" |
    tar -xf - -C "$sdk_dir/freertos/cvitek"
  git -C "$sdk_source_dir" archive --format=tar "HEAD:build/scripts" |
    tar -xf - -C "$sdk_dir/build/scripts"
  git -C "$sdk_source_dir" show "HEAD:build/envsetup_milkv.sh" >"$sdk_dir/build/envsetup_milkv.sh"

  # git apply provides context and hash validation for the BSP patches. The
  # staging repository is disposable and lives under this firmware tree.
  git init --quiet "$sdk_dir"
  git -C "$sdk_dir" add -A
}

stage_licheervnano_environment() {
  local map_output=$rtos_env_dir/output/$rtos_project

  rm -rf "$rtos_env_dir"
  mkdir -p "$map_output"
  cp "$port_root/licheervnano/rtos.config" "$rtos_env_dir/.config"
  python3 "$sdk_dir/build/scripts/mmap_conv.py" --type ld \
    "$port_root/licheervnano/memmap.py" "$map_output/cvi_board_memmap.ld"
}

export_compile_commands() {
  local database=$sdk_dir/freertos/cvitek/build/task/compile_commands.json
  [[ -s $database ]] || return 0
  mkdir -p "$clangd_dir"
  # Native CMake emits native paths, so clangd consumes this database directly.
  cp "$database" "$clangd_dir/compile_commands.json"
}

stage_sdk_worktree
export SG200X_BSP_ROOT=$bsp_root
bash "$port_root/prepare.sh" "$sdk_dir"
trap export_compile_commands EXIT
if [[ ${SG200X_STAGE_ONLY:-0} == 1 ]]; then
  echo "SDK staging completed: $sdk_dir"
  exit 0
fi

toolchain_stamp=$sdk_dir/freertos/cvitek/build/.sg200x-cross-compile
toolchain_id="${SG200X_CROSS_COMPILE:-}"
if [[ ! -f $toolchain_stamp ]] || [[ $(<"$toolchain_stamp") != "$toolchain_id" ]]; then
  # Vendor CMake caches compiler paths. Reconfigure generated SDK build trees
  # when the selected cross compiler changes.
  rm -rf "$sdk_dir/freertos/cvitek/build"
  mkdir -p "${toolchain_stamp%/*}"
  printf '%s\n' "$toolchain_id" >"$toolchain_stamp"
fi

mkdir -p "$output_dir"
stage_licheervnano_environment
export BUILD_PATH=$rtos_env_dir
export CHIP_ARCH=CV181X
export PROJECT_FULLNAME=$rtos_project
export DDR_64MB_SIZE=n
bash "$sdk_dir/freertos/cvitek/build_cv181x.sh"

cp "$sdk_dir/freertos/cvitek/install/bin/cvirtos.elf" "$output_dir/c906-mcu.elf"
cp "$sdk_dir/freertos/cvitek/install/bin/cvirtos.bin" "$output_dir/c906-mcu.bin"
printf '%s\n' "$RTOS_SDK_COMMIT" >"$output_dir/rtos-sdk-commit.txt"

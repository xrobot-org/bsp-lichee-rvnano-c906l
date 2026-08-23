#!/usr/bin/env bash
# Build the runtime-replaceable firmware from the pinned Vendor SDK.
set -euo pipefail

port_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
bsp_root=$(cd "$port_root/../.." && pwd)
source "$port_root/sdk.env"

sdk_dir=${SG200X_DUO_SDK_DIR:-}
sdk_board=${SDK_BOARD:-milkv-duo256m-musl-riscv64-sd}
output_dir=${OUTPUT_DIR:-$bsp_root/build/duo-sdk}
clangd_dir=${CLANGD_OUTPUT_DIR:-$bsp_root/build/clangd}

export_compile_commands() {
  local database=$sdk_dir/freertos/cvitek/build/task/compile_commands.json
  [[ -s $database ]] || return 0
  mkdir -p "$clangd_dir"
  if [[ -n ${SG200X_CLANGD_SDK_DIR:-} ]]; then
    if [[ -n ${SG200X_CROSS_COMPILE:-} ]]; then
      sed -e "s|$sdk_dir|$SG200X_CLANGD_SDK_DIR|g" \
          -e "s|$SG200X_CROSS_COMPILE|${SG200X_CLANGD_CROSS_COMPILE:-$SG200X_CROSS_COMPILE}|g" \
          "$database" >"$clangd_dir/compile_commands.json"
    else
      sed -e "s|$sdk_dir|$SG200X_CLANGD_SDK_DIR|g" \
          "$database" >"$clangd_dir/compile_commands.json"
    fi
  else
    cp "$database" "$clangd_dir/compile_commands.json"
  fi
}

if [[ -z $sdk_dir ]]; then
  echo "Set SG200X_DUO_SDK_DIR to a disposable Duo SDK worktree." >&2
  exit 2
fi

sdk_dir=$(cd "$sdk_dir" && pwd)
actual_commit=$(git -C "$sdk_dir" rev-parse HEAD)
if [[ $actual_commit != "$RTOS_SDK_COMMIT" ]]; then
  echo "SDK commit is $actual_commit; expected $RTOS_SDK_COMMIT." >&2
  exit 1
fi
export SG200X_BSP_ROOT=$bsp_root
bash "$port_root/prepare.sh" "$sdk_dir"
trap export_compile_commands EXIT

if [[ -n ${SG200X_WCH_TOOLCHAIN_DIR:-} ]]; then
  shim_dir=$sdk_dir/.sg200x-wch-toolchain
  rm -rf "$shim_dir"
  mkdir -p "$shim_dir"
  for tool in "$SG200X_WCH_TOOLCHAIN_DIR"/riscv-none-embed-*.exe; do
    [[ -f $tool ]] || continue
    tool_name=${tool##*/}
    ln -s "$tool" "$shim_dir/${tool_name%.exe}"
  done
fi

toolchain_stamp=$sdk_dir/freertos/cvitek/build/.sg200x-cross-compile
toolchain_id="${SG200X_CROSS_COMPILE:-}|${SG200X_WCH_TOOLCHAIN_DIR:-}"
if [[ ! -f $toolchain_stamp ]] || [[ $(<"$toolchain_stamp") != "$toolchain_id" ]]; then
  # Vendor CMake caches compiler paths. Reconfigure generated SDK build trees
  # when the selected cross compiler changes.
  rm -rf "$sdk_dir/freertos/cvitek/build"
  mkdir -p "${toolchain_stamp%/*}"
  printf '%s\n' "$toolchain_id" >"$toolchain_stamp"
fi

mkdir -p "$output_dir"
pushd "$sdk_dir" >/dev/null
set +u
source build/envsetup_milkv.sh "$sdk_board"
build_rtos
set -u
popd >/dev/null

cp "$sdk_dir/freertos/cvitek/install/bin/cvirtos.elf" "$output_dir/c906-mcu.elf"
cp "$sdk_dir/freertos/cvitek/install/bin/cvirtos.bin" "$output_dir/c906-mcu.bin"
printf '%s\n' "$RTOS_SDK_COMMIT" >"$output_dir/rtos-sdk-commit.txt"

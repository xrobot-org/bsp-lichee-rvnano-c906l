#!/usr/bin/env bash
set -euo pipefail

firmware=${1:-}
target=${SG200X_TARGET:-}
ssh_port=${SG200X_SSH_PORT:-}
ssh_password=${SG200X_SSH_PASSWORD:-}
ssh_binary=${SG200X_SSH_COMMAND:-ssh}
sudo_password=${SG200X_SUDO_PASSWORD:-}
dma_int_mux=${SG200X_DMA_INT_MUX:-}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
board_config=${SG200X_BOARD_CONFIG:-"$script_dir/../sdk/sg200x/licheervnano/deploy.conf"}

if [[ ! -f $board_config ]]; then
  echo "Board deployment configuration not found: $board_config" >&2
  exit 2
fi
# Board files are trusted repository configuration. Values can still be
# overridden by exporting the corresponding variable before deployment.
# shellcheck source=/dev/null
source "$board_config"
exclusive_platform_devices=${SG200X_EXCLUSIVE_PLATFORM_DEVICES:-}

if [[ -z $firmware || -z $target ]]; then
  echo "Usage: SG200X_TARGET=root@board-ip $0 <path-to-c906-mcu.elf>" >&2
  exit 2
fi
if [[ ! -f $firmware ]]; then
  echo "Firmware not found: $firmware" >&2
  exit 1
fi
if [[ -n $dma_int_mux && ! $dma_int_mux =~ ^0x[0-9A-Fa-f]{1,8}$ ]]; then
  echo "SG200X_DMA_INT_MUX must be a 32-bit hexadecimal value." >&2
  exit 2
fi
for owner in $exclusive_platform_devices; do
  if [[ ! $owner =~ ^[A-Za-z0-9_.-]+:[A-Za-z0-9_.-]+$ ]]; then
    echo "Invalid exclusive platform device '$owner'; expected driver:device." >&2
    exit 2
  fi
done

# The reference maixcam-sc035hgs image documents debian/rv. Keep that
# image-specific default out of process arguments. Alternate targets can use
# SSH keys or provide credentials explicitly in their environment.
if [[ $target == debian@10.42.0.1 ]]; then
  if [[ -z $ssh_password ]]; then
    ssh_password=rv
  fi
  if [[ -z $sudo_password ]]; then
    sudo_password=rv
  fi
elif [[ -z $sudo_password ]]; then
  echo "Set SG200X_SUDO_PASSWORD for target $target." >&2
  exit 2
fi

ssh_args=()
if [[ -n $ssh_port ]]; then
  ssh_args+=(-p "$ssh_port")
fi
ssh_command=(ssh)
if [[ -n $ssh_password ]]; then
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "sshpass is required when SG200X_SSH_PASSWORD is set." >&2
    exit 2
  fi
  export SSHPASS=$ssh_password
  # The Windows sshpass build cannot drive MSYS OpenSSH's pseudo-terminal.
  # Use the native OpenSSH client when this script runs under Git Bash.
  if [[ -z ${SG200X_SSH_COMMAND:-} && ${OS:-} == Windows_NT &&
        -x /c/Windows/System32/OpenSSH/ssh.exe ]]; then
    ssh_binary=/c/Windows/System32/OpenSSH/ssh.exe
  fi
  ssh_command=(sshpass -e "$ssh_binary")
else
  ssh_command=("$ssh_binary")
fi

remote_tmp=/tmp/c906-mcu.elf.new
"${ssh_command[@]}" "${ssh_args[@]}" "$target" "cat > '$remote_tmp'" <"$firmware"
{
  printf '%s\n' "$sudo_password"
  if [[ -n $dma_int_mux ]]; then
    printf 'dma_int_mux=%q\n' "$dma_int_mux"
  fi
  printf 'exclusive_platform_devices=%q\n' "$exclusive_platform_devices"
  cat <<'REMOTE_SCRIPT'
set -eu
for rproc in /sys/class/remoteproc/remoteproc*; do
  [ -e "$rproc/name" ] || continue
  [ "$(cat "$rproc/name")" = cv181x-c906_1 ] || continue
  if [ "$(cat "$rproc/state")" = running ]; then
    printf stop > "$rproc/state"
  fi
  if [ -n "${dma_int_mux:-}" ]; then
    busybox devmem 0x03000298 32 "$dma_int_mux"
  fi
  for owner in ${exclusive_platform_devices:-}; do
    driver=${owner%%:*}
    device=${owner#*:}
    driver_path=/sys/bus/platform/drivers/$driver
    device_path=/sys/bus/platform/devices/$device
    if [ ! -e "$device_path" ] || [ ! -w "$driver_path/unbind" ]; then
      echo "Cannot claim platform device $device from $driver." >&2
      exit 1
    fi
    if [ -L "$device_path/driver" ]; then
      current_driver=$(basename "$(readlink "$device_path/driver")")
      if [ "$current_driver" != "$driver" ]; then
        echo "Platform device $device is owned by $current_driver, not $driver." >&2
        exit 1
      fi
      printf '%s' "$device" > "$driver_path/unbind"
    fi
    if [ -L "$device_path/driver" ]; then
      echo "Platform device $device remained bound after unbind." >&2
      exit 1
    fi
  done
  install -D -m 0644 /tmp/c906-mcu.elf.new /lib/firmware/c906-mcu.elf
  sync
  printf start > "$rproc/state"
  [ "$(cat "$rproc/state")" = running ] || exit 1
  rm -f /tmp/c906-mcu.elf.new
  exit 0
done
exit 1
REMOTE_SCRIPT
} | "${ssh_command[@]}" "${ssh_args[@]}" "$target" "sudo -S -p '' sh -s"

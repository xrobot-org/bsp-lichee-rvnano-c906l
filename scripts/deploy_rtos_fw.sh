#!/usr/bin/env bash
set -euo pipefail

firmware=${1:-}
target=${SG200X_TARGET:-}
ssh_port=${SG200X_SSH_PORT:-}

if [[ -z $firmware || -z $target ]]; then
  echo "Usage: SG200X_TARGET=root@board-ip $0 <path-to-c906-mcu.elf>" >&2
  exit 2
fi
if [[ ! -f $firmware ]]; then
  echo "Firmware not found: $firmware" >&2
  exit 1
fi

ssh_args=()
if [[ -n $ssh_port ]]; then
  ssh_args+=(-p "$ssh_port")
fi

remote_tmp=/tmp/c906-mcu.elf.new
ssh "${ssh_args[@]}" "$target" "cat > '$remote_tmp'" <"$firmware"
ssh "${ssh_args[@]}" "$target" '
  set -eu
  test -x /usr/bin/rtos-mode
  install -D -m 0644 /tmp/c906-mcu.elf.new /lib/firmware/c906-mcu.elf
  sync
  rtos-mode remoteproc
  for rproc in /sys/class/remoteproc/remoteproc*; do
    [ -e "$rproc/name" ] || continue
    [ "$(cat "$rproc/name")" = cv181x-c906_1 ] || continue
    [ "$(cat "$rproc/state")" = running ] || exit 1
    rm -f /tmp/c906-mcu.elf.new
    exit 0
  done
  exit 1
'

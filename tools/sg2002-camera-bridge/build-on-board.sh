#!/bin/sh
set -eu

cc=${CC:-musl-gcc}
source_file=${1:-bridge.c}
output_file=${2:-sg2002-camera-bridge}
library_dir=${CVI_LIBRARY_DIR:-/mnt/system/usr/lib}

"$cc" -O2 -std=c11 -Wall -Wextra -Werror -D_DEFAULT_SOURCE \
  "$source_file" /usr/lib/libmaix_mmf.a \
  -L"$library_dir" -L"$library_dir/3rd" \
  -Wl,-rpath,"$library_dir:$library_dir/3rd" -Wl,--no-as-needed \
  -lsample -lisp -lisp_algo -lsns_full -lsns_sc035hgs -lae -lawb -laf \
  -lcvi_bin -lcvi_bin_isp -lvi -lvpss -lvenc -lvdec -lvo -lrgn -lsys \
  -lcvi_ive -lmisc -lgdc -lraw_dump -ljson-c -lini -lcli -lpthread -ldl -lm \
  -Wl,--as-needed \
  -o "$output_file"

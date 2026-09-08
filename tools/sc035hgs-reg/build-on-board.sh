#!/bin/sh
set -eu

cc=${CC:-cc}
source_file=${1:-sc035hgs-reg.c}
output_file=${2:-sc035hgs-reg}

"$cc" -O2 -std=c11 -Wall -Wextra -Werror "$source_file" -o "$output_file"

#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/dual_imu_usb_stream_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/tests/stubs" \
    -I"$project_root/App" \
    -I"$project_root/App/imu" \
    -I"$project_root/App/fusion" \
    -I"$project_root/App/protocol" \
    "$project_root/App/protocol/hi91_frame.c" \
    "$project_root/App/protocol/dual_imu_usb_stream.c" \
    "$project_root/tests/test_dual_imu_usb_stream.c" \
    -lm -o "$output"

"$output"

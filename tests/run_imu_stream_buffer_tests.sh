#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/imu_stream_buffer_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/tests/stubs" \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_stream_buffer.c" \
    "$project_root/tests/test_imu_stream_buffer.c" \
    -o "$output"

"$output"

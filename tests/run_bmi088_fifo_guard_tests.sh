#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/bmi088_fifo_guard_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/imu" \
    "$project_root/tests/test_bmi088_fifo_guard.c" \
    -o "$output"

"$output"

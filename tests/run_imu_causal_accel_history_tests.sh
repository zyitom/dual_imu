#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/imu_causal_accel_history_tests"
sanitized_output="${TMPDIR:-/tmp}/imu_causal_accel_history_tests_sanitized"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_causal_accel_history.c" \
    "$project_root/tests/test_imu_causal_accel_history.c" \
    -lm -o "$output"

"$output"

cc -std=c11 -O1 -g -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_causal_accel_history.c" \
    "$project_root/tests/test_imu_causal_accel_history.c" \
    -lm -o "$sanitized_output"

ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
    "$sanitized_output"

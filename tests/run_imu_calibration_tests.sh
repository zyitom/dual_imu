#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/imu_calibration_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$output"

"$output"

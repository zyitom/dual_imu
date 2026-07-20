#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/cross_lane_calibrator_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/fusion" \
    -I"$project_root/App/imu" \
    "$project_root/App/fusion/cross_lane_calibrator.c" \
    "$project_root/tests/test_cross_lane_calibrator.c" \
    -lm -o "$output"

"$output"

#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/dual_imu_estimator_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/fusion" \
    -I"$project_root/App/imu" \
    "$project_root/App/fusion/attitude_mekf.c" \
    "$project_root/App/fusion/dual_imu_estimator.c" \
    "$project_root/App/fusion/imu_preintegrator.c" \
    "$project_root/App/fusion/imu_selector.c" \
    "$project_root/App/imu/imu_geometry.c" \
    "$project_root/tests/test_dual_imu_estimator.c" \
    -lm -o "$output"

"$output"

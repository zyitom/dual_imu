#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "$0")/.." && pwd)"
output="${TMPDIR:-/tmp}/imu_clock_sync_test"

cc -std=c11 -Wall -Wextra -Werror -Wpedantic \
    -I"$project_root/App/imu" \
    "$project_root/tests/test_imu_clock_sync.c" \
    "$project_root/App/imu/imu_clock_sync.c" \
    -lm -o "$output"

"$output"

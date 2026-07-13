#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/imu_selector_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/fusion" \
    "$project_root/App/fusion/imu_selector.c" \
    "$project_root/tests/test_imu_selector.c" \
    -lm -o "$output"

"$output"

#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/fast_attitude_predictor_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/fusion" \
    -I"$project_root/App/imu" \
    "$project_root/App/fusion/fast_attitude_predictor.c" \
    "$project_root/tests/test_fast_attitude_predictor.c" \
    -lm -o "$output"

"$output"

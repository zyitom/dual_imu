#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/attitude_mekf_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/fusion" \
    "$project_root/App/fusion/attitude_mekf.c" \
    "$project_root/tests/test_attitude_mekf.c" \
    -lm -o "$output"

"$output"

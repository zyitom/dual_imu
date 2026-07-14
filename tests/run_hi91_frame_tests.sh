#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
output="${TMPDIR:-/tmp}/hi91_frame_tests"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -I"$project_root/App/protocol" \
    "$project_root/App/protocol/hi91_frame.c" \
    "$project_root/tests/test_hi91_frame.c" \
    -o "$output"

"$output"

#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

PYTHONDONTWRITEBYTECODE=1 python3 "$project_root/tests/test_hi91_capture.py"

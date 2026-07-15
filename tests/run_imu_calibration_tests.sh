#!/usr/bin/env sh
set -eu

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
production_output="${TMPDIR:-/tmp}/imu_calibration_tests_production"
production_unoptimized_object="${TMPDIR:-/tmp}/imu_calibration_production_unoptimized.o"
enabled_output="${TMPDIR:-/tmp}/imu_calibration_tests_high_order_enabled"
g_sensitivity_output="${TMPDIR:-/tmp}/imu_calibration_tests_g_sensitivity_enabled"
production_sanitized_output="${TMPDIR:-/tmp}/imu_calibration_tests_production_sanitized"
enabled_sanitized_output="${TMPDIR:-/tmp}/imu_calibration_tests_high_order_enabled_sanitized"
g_sensitivity_sanitized_output="${TMPDIR:-/tmp}/imu_calibration_tests_g_sensitivity_enabled_sanitized"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=1 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$production_output"

"$production_output"

cc -std=c11 -O0 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=1 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    -c "$project_root/App/imu/imu_calibration.c" \
    -o "$production_unoptimized_object"

qualification_only_symbols='s_stream_state|s_sample_seen|s_temperature_compensation_enabled|select_temperature_use|slew_correction|transform_vector|evaluate_polynomial$|s_gyro_g_sensitivity'
if nm "$production_output" | rg -q "$qualification_only_symbols" || \
   nm "$production_unoptimized_object" | rg -q "$qualification_only_symbols"
then
    echo "production binary retained qualification-only calibration logic" >&2
    exit 1
fi

if cc -std=c11 -fsyntax-only \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=1 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=1 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" >/dev/null 2>&1
then
    echo "production plus high-order build unexpectedly compiled" >&2
    exit 1
fi

if cc -std=c11 -fsyntax-only \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=1 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=1 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" >/dev/null 2>&1
then
    echo "production plus gyro g-sensitivity unexpectedly compiled" >&2
    exit 1
fi

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=0 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=1 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$enabled_output"

"$enabled_output"

cc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=0 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=1 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$g_sensitivity_output"

"$g_sensitivity_output"

cc -std=c11 -O1 -g -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=1 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$production_sanitized_output"

ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
    "$production_sanitized_output"

cc -std=c11 -O1 -g -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=0 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=1 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$enabled_sanitized_output"

ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
    "$enabled_sanitized_output"

cc -std=c11 -O1 -g -Wall -Wextra -Werror -Wpedantic -Wshadow \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DIMU_CALIBRATION_PRODUCTION_BUILD=0 \
    -DIMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0 \
    -DIMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=1 \
    -I"$project_root/App/imu" \
    "$project_root/App/imu/imu_calibration.c" \
    "$project_root/tests/test_imu_calibration.c" \
    -lm -o "$g_sensitivity_sanitized_output"

ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
    "$g_sensitivity_sanitized_output"

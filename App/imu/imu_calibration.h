#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT (4U)
#define IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US UINT64_C(2000000)
#define IMU_CALIBRATION_CORRECTION_SLEW_MAX_STEP_US UINT64_C(10000)
#define IMU_CALIBRATION_DEFAULT_ACCEL_CORRECTION_SLEW_MPS3 (0.0980665f)
#define IMU_CALIBRATION_DEFAULT_GYRO_CORRECTION_SLEW_RAD_S2 \
    (0.00174532925f)
#define IMU_CALIBRATION_MAX_ACCEL_CORRECTION_SLEW_MPS3 (9.80665f)
#define IMU_CALIBRATION_MAX_GYRO_CORRECTION_SLEW_RAD_S2 (0.174532925f)
#define IMU_CALIBRATION_DEFAULT_G_SENSITIVITY_ACCEL_MAX_AGE_US \
    UINT32_C(1000)
#define IMU_CALIBRATION_MAX_G_SENSITIVITY_ACCEL_MAX_AGE_US UINT32_C(5000)
#define IMU_CALIBRATION_MAX_G_SENSITIVITY_ROW_NORM_RAD_S_PER_MPS2 \
    (0.001f)
#define IMU_CALIBRATION_MAX_G_SENSITIVITY_FROBENIUS_NORM_RAD_S_PER_MPS2 \
    (0.0015f)

/*
 * Firmware targets are production builds unless an isolated qualification
 * test explicitly says otherwise. Production and high-order temperature
 * compensation are mutually exclusive at compile time.
 */
#ifndef IMU_CALIBRATION_PRODUCTION_BUILD
#define IMU_CALIBRATION_PRODUCTION_BUILD (1)
#endif

#ifndef IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE
#define IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE (0)
#endif

#ifndef IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE
#define IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE (0)
#endif

#if (IMU_CALIBRATION_PRODUCTION_BUILD != 0) && \
    (IMU_CALIBRATION_PRODUCTION_BUILD != 1)
#error "IMU calibration production build must be 0 or 1"
#endif

#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE != 0) && \
    (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE != 1)
#error "IMU high-order temperature compensation build enable must be 0 or 1"
#endif

#if (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE != 0) && \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE != 1)
#error "IMU gyro g-sensitivity build enable must be 0 or 1"
#endif

#if (IMU_CALIBRATION_PRODUCTION_BUILD == 1) && \
    (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1)
#error "Production firmware cannot include high-order temperature compensation"
#endif

#if (IMU_CALIBRATION_PRODUCTION_BUILD == 1) && \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
#error "Production firmware cannot include gyro g-sensitivity compensation"
#endif

typedef enum
{
    IMU_CALIBRATION_STREAM_ACCEL = 0,
    IMU_CALIBRATION_STREAM_GYRO,
    IMU_CALIBRATION_STREAM_COUNT
} imu_calibration_stream_t;

typedef enum
{
    IMU_CALIBRATION_TEMPERATURE_C0_DISABLED = 0,
    IMU_CALIBRATION_TEMPERATURE_FULL,
    IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED,
    IMU_CALIBRATION_TEMPERATURE_C0_INVALID,
    IMU_CALIBRATION_TEMPERATURE_C0_STALE,
    IMU_CALIBRATION_TEMPERATURE_C0_NONCAUSAL,
    IMU_CALIBRATION_TEMPERATURE_C0_RATE
} imu_calibration_temperature_use_t;

typedef enum
{
    IMU_CALIBRATION_G_SENSITIVITY_DISABLED = 0,
    IMU_CALIBRATION_G_SENSITIVITY_ZERO_MATRIX,
    IMU_CALIBRATION_G_SENSITIVITY_APPLIED,
    IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL,
    IMU_CALIBRATION_G_SENSITIVITY_SATURATED_ACCEL,
    IMU_CALIBRATION_G_SENSITIVITY_SOURCE_MISMATCH,
    IMU_CALIBRATION_G_SENSITIVITY_NONCAUSAL_ACCEL,
    IMU_CALIBRATION_G_SENSITIVITY_STALE_ACCEL
} imu_calibration_g_sensitivity_use_t;

typedef struct
{
    imu_calibration_temperature_use_t temperature_use;
    bool correction_slew_limited;
} imu_calibration_apply_info_t;

typedef struct
{
    uint64_t timestamp_us;
    float accel_body_mps2[3];
    imu_source_t source;
    bool valid;
    bool saturated;
} imu_calibration_accel_evidence_t;

typedef struct
{
    imu_calibration_g_sensitivity_use_t use;
    uint64_t accel_age_us;
} imu_calibration_g_sensitivity_apply_info_t;

typedef struct
{
    uint32_t full_compensation_count;
    uint32_t clamped_compensation_count;
    uint32_t c0_disabled_count;
    uint32_t c0_invalid_count;
    uint32_t c0_stale_count;
    uint32_t c0_noncausal_count;
    uint32_t c0_rate_count;
    uint32_t apply_failure_count;
    uint32_t correction_slew_limited_count[IMU_CALIBRATION_STREAM_COUNT];
    uint32_t temperature_use_transition_count[IMU_CALIBRATION_STREAM_COUNT];
    uint32_t g_sensitivity_applied_count;
    uint32_t g_sensitivity_disabled_count;
    uint32_t g_sensitivity_zero_matrix_count;
    uint32_t g_sensitivity_invalid_accel_count;
    uint32_t g_sensitivity_saturated_accel_count;
    uint32_t g_sensitivity_source_mismatch_count;
    uint32_t g_sensitivity_noncausal_accel_count;
    uint32_t g_sensitivity_stale_accel_count;
    imu_calibration_temperature_use_t last_temperature_use;
    imu_calibration_temperature_use_t
        last_temperature_use_by_stream[IMU_CALIBRATION_STREAM_COUNT];
    bool correction_slew_active[IMU_CALIBRATION_STREAM_COUNT];
    bool temperature_compensation_enabled;
    imu_calibration_g_sensitivity_use_t last_g_sensitivity_use;
    bool gyro_g_sensitivity_enabled;
} imu_calibration_diagnostics_t;

/*
 * Applied after the board's nominal axis rotation:
 *
 *   calibrated = correction_matrix * (nominal_body - bias(temperature))
 *
 * The matrices therefore combine residual sensor scale/non-orthogonality and
 * the measured residual sensor-to-common-body rotation. Bias coefficients are
 * c0..c3 in powers of (temperature-reference_temperature_c).
 */
typedef struct
{
    float accel_correction_matrix[3][3];
    float gyro_correction_matrix[3][3];
    float accel_bias_polynomial_mps2[3]
                                     [IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT];
    float gyro_bias_polynomial_rad_s[3]
                                     [IMU_CALIBRATION_TEMPERATURE_COEFFICIENT_COUNT];
    float reference_temperature_c;
    float minimum_temperature_c;
    float maximum_temperature_c;
    /* Passive plausibility gate; this does not drive a heater or control loop. */
    float maximum_temperature_rate_c_per_s;
    /* Output-frame correction-vector slew limits. */
    float maximum_accel_correction_slew_mps3;
    float maximum_gyro_correction_slew_rad_s2;
    /* Body/output-frame gyro error per calibrated body specific force.
     * Fit the gyro-bias intercept jointly with this matrix. */
    float gyro_g_sensitivity_rad_s_per_mps2[3][3];
    uint32_t maximum_g_sensitivity_accel_age_us;
} imu_calibration_t;

void imu_calibration_initialize(void);
void imu_calibration_reset_all(void);
void imu_calibration_default(imu_calibration_t *calibration);
bool imu_calibration_validate(const imu_calibration_t *calibration);
/* Qualification builds reject coefficient changes after either stream has
 * seen a sample. Production uses the caller-layer pre-stream gate. */
bool imu_calibration_set(imu_source_t source,
                         const imu_calibration_t *calibration);
bool imu_calibration_get(imu_source_t source,
                         imu_calibration_t *calibration);
bool imu_calibration_has_custom(imu_source_t source);
/* Coefficient loading never enables high-order temperature compensation. A
 * true request is rejected unless this is an isolated qualification build
 * with custom coefficients loaded before either stream has seen a sample. */
bool imu_calibration_set_temperature_compensation_enabled(imu_source_t source,
                                                          bool enabled);
bool imu_calibration_temperature_compensation_enabled(imu_source_t source);
/* A true request also requires custom coefficients before the first sample. */
bool imu_calibration_set_gyro_g_sensitivity_enabled(imu_source_t source,
                                                    bool enabled);
bool imu_calibration_gyro_g_sensitivity_enabled(imu_source_t source);
bool imu_calibration_get_diagnostics(
    imu_source_t source,
    imu_calibration_diagnostics_t *diagnostics);

bool imu_calibration_apply_accel(imu_source_t source,
                                 uint64_t sample_timestamp_us,
                                 float temperature_c,
                                 uint64_t temperature_timestamp_us,
                                 bool temperature_valid,
                                 float accel_mps2[3],
                                 imu_calibration_apply_info_t *info);
bool imu_calibration_apply_gyro(imu_source_t source,
                                uint64_t sample_timestamp_us,
                                float temperature_c,
                                uint64_t temperature_timestamp_us,
                                bool temperature_valid,
                                float gyro_rad_s[3],
                                imu_calibration_apply_info_t *info);
/* Second stage: gyro_rad_s must already have temperature/matrix calibration. */
bool imu_calibration_apply_gyro_g_sensitivity(
    imu_source_t source,
    uint64_t gyro_timestamp_us,
    const imu_calibration_accel_evidence_t *accel_evidence,
    float gyro_rad_s[3],
    imu_calibration_g_sensitivity_apply_info_t *info);

#ifdef __cplusplus
}
#endif

#endif
